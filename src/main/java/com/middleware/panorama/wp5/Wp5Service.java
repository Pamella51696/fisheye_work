package com.middleware.panorama.wp5;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * WP-5 entry point. One call wires everything onto an existing HttpServer:
 *
 *   Wp5Service wp5 = Wp5Service.attach(server, cameraNames, W, H, overlap);
 *
 * Endpoints
 *   GET  /api/v1/signals?topics=steering,curb   Server-Sent-Events live stream (Android uses this)
 *   GET  /api/v1/steering                       latest steering JSON (polling / debugging)
 *   GET  /api/v1/curb                           latest curb-zone JSON (polling / debugging)
 *   POST /api/v1/ingest/steering                vehicle gateway -> middleware (steering angle)
 *   POST /api/v1/ingest/range                   vehicle gateway -> middleware (distance sensors)
 *   GET  /wp5                                   tiny live debug page (open in a browser)
 */
public final class Wp5Service {

    private final Wp5Config cfg;
    private final SteeringModel steeringModel;
    private final CurbModel curbModel;
    private final SignalHub hub = new SignalHub();
    private final ScheduledExecutorService ticker = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "wp5-ticker"); t.setDaemon(true); return t;
    });
    private final AtomicLong steeringSeq = new AtomicLong();
    private final AtomicLong curbSeq = new AtomicLong();

    // steering bookkeeping (guarded by 'this')
    private SteeringState lastPublishedSteering;
    private long lastSteeringInputMs;
    private String lastCurbSignature = "";
    private long lastCurbPublishMs;
    private VehicleSignalManager signalManager;

    private Wp5Service(Wp5Config cfg, String[] cameraNames, int camW, int camH, int overlap) {
        this.cfg = cfg;
        this.steeringModel = new SteeringModel(cfg);
        this.curbModel = new CurbModel(cfg, cameraNames, camW, camH, overlap);
    }

    public static Wp5Service attach(HttpServer server, String[] cameraNames,
                                    int camW, int camH, int overlap) {
        Wp5Config cfg = Wp5Config.load();
        Wp5Service s = new Wp5Service(cfg, cameraNames, camW, camH, overlap);
        server.createContext("/api/v1/signals", s.new SseHandler());
        server.createContext("/api/v1/steering", s.new LatestHandler("steering"));
        server.createContext("/api/v1/curb", s.new LatestHandler("curb"));
        server.createContext("/api/v1/ingest/steering", s.new IngestHandler(true));
        server.createContext("/api/v1/ingest/range", s.new IngestHandler(false));
        server.createContext("/api/v1/profile", s.new ProfileHandler());
        server.createContext("/api/v1/signal-source", s.new SignalSourceHandler());
        server.createContext("/wp5", s.new DebugPageHandler());
        server.createContext("/wp5/viewer", s.new ViewerPageHandler());
        System.out.println("[WP5] profile: " + cfg.profile.name + "  (from " + cfg.profileSource + ")");
        s.ticker.scheduleAtFixedRate(s::tick, 100, 100, TimeUnit.MILLISECONDS);
        s.signalManager = new VehicleSignalManager(cfg, s);
        s.signalManager.start();
        System.out.println("[WP5] vehicle signal source: " + cfg.signalSource
                + " (override: -Dwp5.signalSource=AUTO|UDP|SIMULATOR)");
        System.out.println("[WP5] Signals (SSE) →  /api/v1/signals    Debug page →  /wp5    Android stand-in viewer →  /wp5/viewer");
        return s;
    }

    public void stop() {
        ticker.shutdownNow();
        if (signalManager != null) {
            signalManager.close();
        }
    }

    // ---------------------------------------------------------------- inputs

    /** Called with a raw vehicle-bus value. Safe to call from any thread, at any rate. */
    public synchronized void onSteeringInput(double steeringWheelDeg, String gear, double speedKph) {
        long now = System.currentTimeMillis();
        lastSteeringInputMs = now;
        SteeringState next = steeringModel.compute(steeringSeq.get() + 1, now, steeringWheelDeg, gear, speedKph);
        if (steeringModel.shouldPublish(lastPublishedSteering, next, now)) {
            steeringSeq.incrementAndGet();
            lastPublishedSteering = next;
            hub.publish("steering", next.toJson());
        }
    }

    public void onRangeJson(String body) {
        curbModel.ingestJson(body, System.currentTimeMillis());
    }

    // ------------------------------------------------------------------ tick (10 Hz)

    private synchronized void tick() {
        try {
            long now = System.currentTimeMillis();

            // steering heartbeat + stale marking
            if (lastPublishedSteering != null && now - lastPublishedSteering.tsMs >= cfg.heartbeatMs) {
                boolean fresh = now - lastSteeringInputMs <= 2 * cfg.heartbeatMs;
                lastPublishedSteering = lastPublishedSteering.reissue(steeringSeq.incrementAndGet(), now, fresh);
                hub.publish("steering", lastPublishedSteering.toJson());
            }

            // curb zones: publish when something visible changed, else heartbeat
            if (curbModel.hasSensors()) {
                CurbModel.CurbState st = curbModel.snapshot(curbSeq.get() + 1, now);
                boolean changed = !st.signature.equals(lastCurbSignature);
                if (changed || now - lastCurbPublishMs >= cfg.heartbeatMs) {
                    curbSeq.incrementAndGet();
                    lastCurbSignature = st.signature;
                    lastCurbPublishMs = now;
                    hub.publish("curb", st.json);
                }
            }
        } catch (Throwable t) {
            System.err.println("[WP5] tick error: " + t); // never let the scheduler die
        }
    }

    // --------------------------------------------------------------- HTTP

    private static void send(HttpExchange ex, int code, String type, String body) throws IOException {
        byte[] b = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", type);
        ex.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
        ex.getResponseHeaders().set("Cache-Control", "no-cache");
        ex.sendResponseHeaders(code, b.length);
        try (OutputStream os = ex.getResponseBody()) { os.write(b); }
    }

    /** Long-lived SSE connection: holds one executor thread per client (like /stitch does). */
    private final class SseHandler implements HttpHandler {
        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
            Set<String> topics = new HashSet<>();
            String q = ex.getRequestURI().getQuery();
            if (q != null) for (String kv : q.split("&"))
                if (kv.startsWith("topics=")) topics.addAll(Arrays.asList(kv.substring(7).split(",")));

            ex.getResponseHeaders().set("Content-Type", "text/event-stream; charset=UTF-8");
            ex.getResponseHeaders().set("Cache-Control", "no-cache");
            ex.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
            ex.sendResponseHeaders(200, 0);

            SignalHub.Subscriber sub = hub.subscribe(topics);
            try (OutputStream out = ex.getResponseBody()) {
                // current state first, so a freshly started app is correct immediately
                for (String t : new String[]{"steering", "curb"}) {
                    String last = hub.latest(t);
                    if (last != null && (topics.isEmpty() || topics.contains(t))) {
                        out.write(SignalHub.frame(t, last).getBytes(StandardCharsets.UTF_8));
                    }
                }
                out.flush();
                while (true) {
                    String msg = sub.queue().poll(10, TimeUnit.SECONDS);
                    out.write((msg != null ? msg : ": ping\n\n").getBytes(StandardCharsets.UTF_8));
                    out.flush();
                }
            } catch (IOException | InterruptedException e) {
                // client went away — normal
            } finally {
                hub.unsubscribe(sub);
            }
        }
    }

    private final class LatestHandler implements HttpHandler {
        private final String topic;
        LatestHandler(String topic) { this.topic = topic; }
        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
            String j = hub.latest(topic);
            if (j == null) { ex.sendResponseHeaders(204, -1); return; }
            send(ex, 200, "application/json; charset=UTF-8", j);
        }
    }

    private final class IngestHandler implements HttpHandler {
        private final boolean steering;
        IngestHandler(boolean steering) { this.steering = steering; }
        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"POST".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
            if (!cfg.ingestToken.isEmpty()
                    && !cfg.ingestToken.equals(ex.getRequestHeaders().getFirst("X-Api-Key"))) {
                ex.sendResponseHeaders(401, -1); return;
            }
            byte[] raw = ex.getRequestBody().readNBytes(64 * 1024);
            String body = new String(raw, StandardCharsets.UTF_8);
            try {
                if (steering) {
                    double a = Json.getNum(body, "steeringWheelDeg", Double.NaN);
                    if (Double.isNaN(a)) { send(ex, 400, "application/json", "{\"error\":\"steeringWheelDeg missing\"}"); return; }
                    if (signalManager != null) {
                        signalManager.onHttpSteering(a, Json.getStr(body, "gear", "D"),
                                Json.getNum(body, "speedKph", 0));
                    } else {
                        onSteeringInput(a, Json.getStr(body, "gear", "D"), Json.getNum(body, "speedKph", 0));
                    }
                    send(ex, 200, "application/json", "{\"ok\":true}");
                } else {
                    int n = curbModel.ingestJson(body, System.currentTimeMillis());
                    send(ex, n > 0 ? 200 : 400, "application/json", "{\"accepted\":" + n + "}");
                }
            } catch (RuntimeException e) {
                send(ex, 400, "application/json", "{\"error\":\"bad request\"}");
            }
        }
    }

    /** Android can read the active profile (node names, axes...) instead of hard-coding it. */
    private final class ProfileHandler implements HttpHandler {
        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
            send(ex, 200, "application/json; charset=UTF-8", cfg.profile.rawJson);
        }
    }

    private final class SignalSourceHandler implements HttpHandler {
        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
            String body = signalManager != null ? signalManager.statusJson()
                    : "{\"signalSource\":\"" + cfg.signalSource.name() + "\"}";
            send(ex, 200, "application/json; charset=UTF-8", body);
        }
    }

    /** Browser stand-in for the Android app + fake vehicle: panorama overlay, top view, sliders. */
    private final class ViewerPageHandler implements HttpHandler {
        @Override public void handle(HttpExchange ex) throws IOException {
            try (java.io.InputStream in = Wp5Service.class.getResourceAsStream("/wp5/viewer.html")) {
                if (in == null) { send(ex, 404, "text/plain", "viewer.html not on classpath (src/main/resources/wp5). Run: mvn clean compile"); return; }
                send(ex, 200, "text/html; charset=UTF-8", new String(in.readAllBytes(), StandardCharsets.UTF_8));
            }
        }
    }

    private final class DebugPageHandler implements HttpHandler {
        @Override public void handle(HttpExchange ex) throws IOException {
            send(ex, 200, "text/html; charset=UTF-8", DEBUG_HTML);
        }
    }

    private static final String DEBUG_HTML = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        + "<meta name='viewport' content='width=device-width,initial-scale=1'><title>WP5 live</title>"
        + "<style>body{background:#0a0a0f;color:#e0e0e0;font-family:sans-serif;padding:16px}"
        + "h2{color:#7ec8e3;font-weight:300}.z{display:inline-block;margin:4px;padding:10px 14px;border-radius:6px;color:#000;min-width:150px}"
        + "pre{background:#15151f;padding:10px;border-radius:6px;overflow:auto;font-size:12px}</style></head><body>"
        + "<p><a style='color:#7ec8e3' href='/wp5/viewer'>Open the full viewer (panorama overlay, top view, fake-vehicle sliders)</a></p>"
        + "<h2>Steering</h2><div id='s'>waiting…</div><pre id='sj'></pre>"
        + "<h2>Curb / obstacle zones</h2><div id='c'>waiting…</div>"
        + "<script>const es=new EventSource('/api/v1/signals');"
        + "es.addEventListener('steering',e=>{const d=JSON.parse(e.data);"
        + "document.getElementById('s').textContent='wheel '+d.steeringWheelDeg+'° | L '+d.leftWheelDeg+'° R '+d.rightWheelDeg+'° | '+d.direction+' | gear '+d.gear+(d.valid?'':' (STALE)');"
        + "document.getElementById('sj').textContent=JSON.stringify(d).slice(0,300)+'…';});"
        + "es.addEventListener('curb',e=>{const d=JSON.parse(e.data);"
        + "document.getElementById('c').innerHTML=d.zones.map(z=>\"<span class='z' style='background:\"+z.color+\"'>\"+z.pos+'<br>'+z.zone+(z.distanceM!==null?' '+z.distanceM+' m':'')+'<br>pano x='+z.pano.x+' w='+z.pano.w+'</span>').join('');});"
        + "</script></body></html>";
}
