import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
 
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.Executors;
 
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import configuration.CurbConfig;
import output.CurbDetectionResult;
import output.FusedCurbResult;
import perception.curb.CurbPerceptionPipeline;
import surround.analytics.CurbDebugOverlay;
import surround.analytics.CurbDetectionService;
import surround.analytics.CurbResultFilter;
import surround.runtime.CameraCaptureManager;
import surround.runtime.CameraIo;
import surround.runtime.FrameDistributor;
import surround.runtime.FrameSnapshot;
import surround.runtime.SurroundPipeline;
import surround.signal.SignalPublisher;
import surround.signal.SignalWebSocketServer;
import surround.vehicle.UdpVehicleSignalService;
import surround.vehicle.VehicleSignalConfig;
import surround.vehicle.VehicleSignalSimulator;
 
/**
 * Multi-camera panoramic projection with a shared vehicle-frame horizon.
 *
 * Ray-based surround pipeline (see {@code config/rig.json}).
 * Fisheye → camera model → 3D ray → vehicle frame → panorama → feather blend.
 * Rectilinear corrected feeds: {@code /corrected/<role>}.
 * Curb + vehicle signals: {@code /api/signals} (async analytics, not in video loop).
 */
public class VideoStreamingServer {
 
    private static final int DEFAULT_PORT = 9090;
    private static final Path RIG_CONFIG = Paths.get("config", "rig.json");
    private static final Path CURB_CONFIG = Paths.get("config", "curb.json");
    private static final String[] CAM_ROLE = { "left", "front", "right", "rear" };
 
    public static void main(String[] args) throws IOException {
        VehicleSignalConfig vehicleConfig = VehicleSignalConfig.parse(args, DEFAULT_PORT);
        int port = vehicleConfig.httpPort;
 
        Path folder = Paths.get(".").toAbsolutePath().normalize();
        Path[] videos = discoverClips(folder);
        if (videos == null) {
            return;
        }
 
        try {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        } catch (UnsatisfiedLinkError e) {
            System.err.println("OpenCV native library not found: " + e.getMessage());
            return;
        }
        CameraIo.loadFfmpegPlugin();

        SurroundPipeline pipeline;
        try {
            pipeline = SurroundPipeline.load(RIG_CONFIG);
        } catch (IOException e) {
            System.err.println("Missing or invalid " + RIG_CONFIG.toAbsolutePath()
                    + " — run: python3 calibration/synthetic_calibrate.py .");
            System.err.println(e.getMessage());
            return;
        }

        FrameDistributor frameDistributor = new FrameDistributor();
        CameraCaptureManager captureManager =
                new CameraCaptureManager(videos, frameDistributor);
        captureManager.start();

        CurbDetectionService curbService = null;
        SignalPublisher signalPublisher = null;
        UdpVehicleSignalService vehicleService =
                new UdpVehicleSignalService(new VehicleSignalSimulator(), vehicleConfig);
        vehicleService.start();
        try {
            CurbConfig curbConfig = CurbConfig.load(CURB_CONFIG);
            CurbPerceptionPipeline curbPipeline =
                    new CurbPerceptionPipeline(curbConfig, pipeline);
            CurbResultFilter filter = new CurbResultFilter(0.08, 500);
            curbService = new CurbDetectionService(frameDistributor, curbPipeline, filter);
            curbService.start();
            signalPublisher = new SignalPublisher(curbService, vehicleService, 1.0);
            System.out.println("Async curb analytics enabled (" + CURB_CONFIG + ")");
        } catch (IOException e) {
            System.err.println("Curb config missing; curb signals UNAVAILABLE: " + e.getMessage());
            signalPublisher = new SignalPublisher(null, vehicleService, 1.0);
        }

        final SignalPublisher signals = signalPublisher;
        final UdpVehicleSignalService vehicle = vehicleService;
        final CurbDetectionService curbSvc = curbService;

        HttpServer server = HttpServer.create(new InetSocketAddress(port), 0);
        server.createContext("/stitch", new StitchHandler(pipeline, frameDistributor));
        for (String role : CAM_ROLE) {
            server.createContext("/corrected/" + role,
                    new CorrectedHandler(pipeline, frameDistributor, role));
        }
        if (curbSvc != null) {
            for (String role : CAM_ROLE) {
                server.createContext("/debug/curb/" + role,
                        new DebugCurbHandler(pipeline, frameDistributor, curbSvc, role));
            }
        }
        server.createContext("/api/signals", new SignalsHandler(signals, false));
        server.createContext("/api/signals/stream", new SignalsHandler(signals, true));
        server.createContext("/api/vehicle/simulate", new VehicleSimulateHandler(vehicle));
        server.createContext("/play",   new PlayerPageHandler(port, curbSvc != null));
        server.setExecutor(Executors.newFixedThreadPool(4));
        server.start();

        SignalWebSocketServer wsServer = null;
        if (vehicleConfig.signalWebSocketPort > 0) {
            wsServer = new SignalWebSocketServer(
                    vehicleConfig.signalWebSocketPort, signals, vehicleConfig.signalWebSocketHz);
            wsServer.start();
        }
 
        System.out.println("Server started  ->  http://localhost:" + port + "/play");
        System.out.println("  Panorama (no curb overlay): /stitch");
        if (curbSvc != null) {
            System.out.println("  Curb DEV overlay: /debug/curb/right  (also left, front, rear)");
            System.out.println("  Curb JSON: /api/signals");
        }
        if (vehicleConfig.signalWebSocketPort > 0) {
            System.out.println("  Signal WebSocket: ws://localhost:"
                    + vehicleConfig.signalWebSocketPort + "/signals");
        }
        for (int i = 0; i < CAM_ROLE.length; i++) {
            System.out.println("  " + CAM_ROLE[i] + " = " + videos[i]);
        }
    }
 
    /**
     * Finds left/front/right/rear clips in {@code folder}. Prefers {@code name_1.mp4}
     * then {@code name.mp4}/{@code .mov}, then any file whose name contains the role.
     */
    static Path[] discoverClips(Path folder) {
        Path[] found = new Path[4];
        for (int i = 0; i < CAM_ROLE.length; i++) {
            found[i] = findClip(folder, CAM_ROLE[i]);
            if (found[i] == null) {
                System.err.println("No " + CAM_ROLE[i]
                        + " clip in " + folder
                        + " (expected e.g. " + CAM_ROLE[i] + "_1.mp4 or "
                        + CAM_ROLE[i] + ".mp4)");
                return null;
            }
            found[i] = CameraIo.ensureDecodable(found[i]);
        }
        return found;
    }
 
    static Path findClip(Path folder, String role) {
        String r = role.toLowerCase(Locale.ROOT);
        String[] preferred = {
            r + "_1.mp4", r + ".mp4", r + "_1.mov", r + ".mov",
            r + "_1.MP4", r + ".MP4"
        };
        for (String name : preferred) {
            Path p = folder.resolve(name);
            if (isUsableFile(p)) {
                return p;
            }
        }
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(folder)) {
            Path fallback = null;
            for (Path p : stream) {
                if (!isUsableFile(p)) continue;
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                if (!(n.endsWith(".mp4") || n.endsWith(".mov") || n.endsWith(".avi"))) {
                    continue;
                }
                if (n.startsWith(r + "_") || n.startsWith(r + ".") || n.equals(r + ".mp4")) {
                    return p;
                }
                if (fallback == null && n.contains(r)) {
                    fallback = p;
                }
            }
            return fallback;
        } catch (IOException e) {
            return null;
        }
    }
 
    private static class StitchHandler implements HttpHandler {
        private final SurroundPipeline pipeline;
        private final FrameDistributor distributor;

        StitchHandler(SurroundPipeline pipeline, FrameDistributor distributor) {
            this.pipeline = pipeline;
            this.distributor = distributor;
        }

        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
                ex.sendResponseHeaders(405, -1);
                return;
            }
            streamMjpeg(ex, pipeline, distributor, false, -1);
        }
    }

    private static class SignalsHandler implements HttpHandler {
        private final SignalPublisher publisher;
        private final boolean stream;

        SignalsHandler(SignalPublisher publisher, boolean stream) {
            this.publisher = publisher;
            this.stream = stream;
        }

        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
                ex.sendResponseHeaders(405, -1);
                return;
            }
            if (stream) {
                ex.getResponseHeaders().set("Content-Type", "application/x-ndjson; charset=UTF-8");
                ex.sendResponseHeaders(200, 0);
                try (OutputStream out = ex.getResponseBody()) {
                    while (true) {
                        String line = publisher.toJson();
                        out.write((line + "\n").getBytes(StandardCharsets.UTF_8));
                        out.flush();
                        Thread.sleep(100);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            } else {
                byte[] bytes = publisher.toJson().getBytes(StandardCharsets.UTF_8);
                ex.getResponseHeaders().set("Content-Type", "application/json; charset=UTF-8");
                ex.sendResponseHeaders(200, bytes.length);
                try (OutputStream out = ex.getResponseBody()) {
                    out.write(bytes);
                }
            }
        }
    }

    private static class VehicleSimulateHandler implements HttpHandler {
        private final UdpVehicleSignalService vehicleService;

        VehicleSimulateHandler(UdpVehicleSignalService vehicleService) {
            this.vehicleService = vehicleService;
        }

        @Override public void handle(HttpExchange ex) throws IOException {
            String method = ex.getRequestMethod();
            if (!"POST".equalsIgnoreCase(method) && !"GET".equalsIgnoreCase(method)) {
                ex.sendResponseHeaders(405, -1);
                return;
            }
            String scenario = queryParam(ex.getRequestURI().getQuery(), "scenario");
            if (scenario == null || scenario.isEmpty()) {
                scenario = "RIGHT_TURN";
            }
            vehicleService.simulator().startScenario(scenario);
            byte[] body = ("{\"started\":true,\"scenario\":\"" + scenario + "\"}")
                    .getBytes(StandardCharsets.UTF_8);
            ex.getResponseHeaders().set("Content-Type", "application/json; charset=UTF-8");
            ex.sendResponseHeaders(200, body.length);
            try (OutputStream out = ex.getResponseBody()) {
                out.write(body);
            }
        }

        private static String queryParam(String query, String key) {
            if (query == null) {
                return null;
            }
            for (String part : query.split("&")) {
                int eq = part.indexOf('=');
                if (eq > 0 && part.substring(0, eq).equals(key)) {
                    return part.substring(eq + 1);
                }
            }
            return null;
        }
    }

    private static class DebugCurbHandler implements HttpHandler {
        private final SurroundPipeline pipeline;
        private final FrameDistributor distributor;
        private final CurbDetectionService curbService;
        private final String role;

        DebugCurbHandler(SurroundPipeline pipeline, FrameDistributor distributor,
                         CurbDetectionService curbService, String role) {
            this.pipeline = pipeline;
            this.distributor = distributor;
            this.curbService = curbService;
            this.role = role;
        }

        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
                ex.sendResponseHeaders(405, -1);
                return;
            }
            int idx = pipeline.cameraIndex(role);
            if (idx < 0) {
                ex.sendResponseHeaders(404, -1);
                return;
            }
            ex.getResponseHeaders().set("Content-Type",
                    "multipart/x-mixed-replace; boundary=frame");
            ex.sendResponseHeaders(200, 0);
            Mat rect = new Mat();
            long lastSeq = 0;
            try (OutputStream out = ex.getResponseBody()) {
                while (true) {
                    FrameSnapshot snap;
                    try {
                        snap = distributor.awaitVideoFrame(lastSeq);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                    try {
                        pipeline.rectify(idx, snap.fisheyeBgr[idx], rect);
                        FusedCurbResult fused = curbService.lastFusedResult();
                        CurbDetectionResult cam = fused == null
                                ? CurbDetectionResult.noDetection(snap.timestampMs, role)
                                : pickCamera(fused, role);
                        CurbDebugOverlay.draw(rect, cam);
                        writeFrame(out, encodeJpeg(rect));
                        lastSeq = snap.sequence;
                    } finally {
                        snap.release();
                    }
                }
            }
        }
    }

    private static CurbDetectionResult pickCamera(FusedCurbResult fused, String role) {
        for (CurbDetectionResult c : fused.perCamera) {
            if (c.cameraId.equalsIgnoreCase(role)) {
                return c;
            }
        }
        return CurbDetectionResult.noDetection(fused.timestampMs, role);
    }

    private static class CorrectedHandler implements HttpHandler {
        private final SurroundPipeline pipeline;
        private final FrameDistributor distributor;
        private final String role;

        CorrectedHandler(SurroundPipeline pipeline, FrameDistributor distributor, String role) {
            this.pipeline = pipeline;
            this.distributor = distributor;
            this.role = role;
        }

        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
                ex.sendResponseHeaders(405, -1);
                return;
            }
            int idx = pipeline.cameraIndex(role);
            if (idx < 0) {
                ex.sendResponseHeaders(404, -1);
                return;
            }
            streamMjpeg(ex, pipeline, distributor, true, idx);
        }
    }

    private static void streamMjpeg(HttpExchange ex,
                                    SurroundPipeline pipeline,
                                    FrameDistributor distributor,
                                    boolean rectilinear,
                                    int cameraIndex) throws IOException {
        ex.getResponseHeaders().set("Content-Type",
                "multipart/x-mixed-replace; boundary=frame");
        ex.sendResponseHeaders(200, 0);
        Mat rect = new Mat();
        long lastSeq = 0;
        try (OutputStream out = ex.getResponseBody()) {
            while (true) {
                FrameSnapshot snap;
                try {
                    snap = distributor.awaitVideoFrame(lastSeq);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
                try {
                    Mat output;
                    if (rectilinear) {
                        pipeline.rectify(cameraIndex, snap.fisheyeBgr[cameraIndex], rect);
                        output = rect;
                    } else {
                        output = pipeline.stitchPanorama(snap.fisheyeBgr);
                    }
                    writeFrame(out, encodeJpeg(output));
                    if (!rectilinear) {
                        output.release();
                    }
                    lastSeq = snap.sequence;
                } finally {
                    snap.release();
                }
            }
        }
    }
 
    private static class PlayerPageHandler implements HttpHandler {
        private final int port;
        private final boolean curbEnabled;

        PlayerPageHandler(int port, boolean curbEnabled) {
            this.port = port;
            this.curbEnabled = curbEnabled;
        }

        @Override public void handle(HttpExchange ex) throws IOException {
            String curbPanel = "";
            if (curbEnabled) {
                curbPanel = ""
                    + "<div id='curbStatus' class='curb-status'>Curb: loading…</div>"
                    + "<div class='split'>"
                    + "  <div class='pano-wrap'><img src='/stitch' alt='panorama'></div>"
                    + "  <div class='pano-wrap'><img src='/debug/curb/right' alt='curb debug right'></div>"
                    + "</div>"
                    + "<p class='hint'>Left: stitch (no overlay). Right: dev curb overlay on corrected camera. "
                    + "Production Android draws zones from <code>/api/signals</code>.</p>"
                    + "<script>"
                    + "async function poll(){try{const r=await fetch('/api/signals');"
                    + "const j=await r.json();const c=j.curb||{};"
                    + "const z=c.zone||c.status||'—';"
                    + "const el=document.getElementById('curbStatus');"
                    + "el.textContent=c.detected?('Curb '+z+' @ '+c.distanceMeters+'m ('+c.camera+')'):('Curb: '+ (c.status||'none'));"
                    + "el.className='curb-status zone-'+(c.zone||'UNKNOWN');"
                    + "}catch(e){}} setInterval(poll,500); poll();"
                    + "</script>";
            } else {
                curbPanel = "<div class='pano-wrap'><img src='/stitch' alt='panorama'></div>";
            }
            String html = "<!DOCTYPE html><html lang='en'><head>"
                + "<meta charset='UTF-8'>"
                + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                + "<title>Surround + curb debug</title>"
                + "<style>"
                + "*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }"
                + "html, body { height: 100%; background: #0a0a0f; color: #e0e0e0;"
                + "  font-family: 'Segoe UI', sans-serif; }"
                + ".container { display: flex; flex-direction: column;"
                + "  height: 100vh; padding: 12px; gap: 8px; }"
                + "h1 { font-size: 1.1rem; font-weight: 400; color: #7ec8e3; text-align: center; }"
                + ".split { display: flex; gap: 8px; flex: 1; min-height: 0; }"
                + ".pano-wrap { flex: 1; min-width: 0; border: 1px solid #2a2a3a; border-radius: 8px;"
                + "  overflow: hidden; display: flex; align-items: center; justify-content: center; }"
                + ".pano-wrap img { width: 100%; height: 100%; object-fit: contain; }"
                + ".curb-status { text-align: center; padding: 8px; border-radius: 6px; background: #1a1a24; }"
                + ".zone-GREEN{background:#0d2a14;color:#8f8;} .zone-YELLOW{background:#2a2208;color:#fd8;}"
                + ".zone-RED{background:#2a0a0a;color:#f88;} .hint{font-size:0.8rem;color:#888;text-align:center;}"
                + "code{color:#9cf;}"
                + "</style></head><body>"
                + "<div class='container'>"
                + "<h1>Surround view (port " + port + ")</h1>"
                + curbPanel
                + "</div></body></html>";
            byte[] bytes = html.getBytes(StandardCharsets.UTF_8);
            ex.getResponseHeaders().set("Content-Type", "text/html; charset=UTF-8");
            ex.sendResponseHeaders(200, bytes.length);
            try (OutputStream os = ex.getResponseBody()) { os.write(bytes); }
        }
    }
 
 
    static Path ensureDecodable(Path requested) {
        Path mp4 = siblingWithExt(requested, ".mp4");
        if (isUsableFile(mp4)) {
            Path a = mp4.toAbsolutePath().normalize();
            Path b = requested.toAbsolutePath().normalize();
            if (!a.equals(b)) {
                System.out.println("Using " + mp4.getFileName() + " (H.264) instead of "
                        + requested.getFileName());
            }
            return mp4;
        }
        if (!isUsableFile(requested)) {
            return requested;
        }
        if (transcodeToH264(requested, mp4) && isUsableFile(mp4)) {
            return mp4;
        }
        System.err.println("Cannot decode " + requested.getFileName()
                + " (PNG-in-MOV). Install ffmpeg on PATH and re-run, or convert:");
        System.err.println("  ffmpeg -y -i " + requested.getFileName()
                + " -c:v libx264 -pix_fmt yuv420p -an " + mp4.getFileName());
        return requested;
    }
 
    static Path siblingWithExt(Path requested, String ext) {
        String name = requested.getFileName().toString();
        int dot = name.lastIndexOf('.');
        String stem = dot >= 0 ? name.substring(0, dot) : name;
        Path dir = requested.toAbsolutePath().getParent();
        if (dir == null) {
            dir = Paths.get(".");
        }
        return dir.resolve(stem + ext);
    }
 
    static boolean isUsableFile(Path path) {
        try {
            return path != null && Files.isRegularFile(path) && Files.size(path) > 0;
        } catch (IOException e) {
            return false;
        }
    }
 
    static boolean transcodeToH264(Path src, Path dst) {
        System.out.println("Transcoding " + src.getFileName() + " -> " + dst.getFileName()
                + " (PNG MOV cannot be decoded by OpenCV FFmpeg/MSMF)");
        ProcessBuilder pb = new ProcessBuilder(
                "ffmpeg", "-hide_banner", "-y",
                "-i", src.toAbsolutePath().toString(),
                "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p",
                "-an", dst.toAbsolutePath().toString());
        pb.inheritIO();
        try {
            int code = pb.start().waitFor();
            if (code == 0 && isUsableFile(dst)) {
                System.out.println("Transcode OK: " + dst.getFileName());
                return true;
            }
            System.err.println("ffmpeg exited with code " + code);
        } catch (IOException e) {
            System.err.println("ffmpeg not found on PATH: " + e.getMessage());
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            System.err.println("ffmpeg transcode interrupted");
        }
        try {
            Files.deleteIfExists(dst);
        } catch (IOException ignored) {
        }
        return false;
    }
 
    static void loadFfmpegPlugin() {
        String[] dllNames = {
            "opencv_videoio_ffmpeg490_64.dll",
            "opencv_videoio_ffmpeg4.dll",
            "opencv_videoio_ffmpeg.dll"
        };
        List<Path> dirs = new ArrayList<>();
        String libPath = System.getProperty("java.library.path", "");
        for (String dir : libPath.split(File.pathSeparator)) {
            if (dir == null || dir.trim().isEmpty()) continue;
            Path p = Paths.get(dir.trim()).toAbsolutePath().normalize();
            dirs.add(p);
            if (p.getParent() != null) {
                dirs.add(p.getParent());
                if (p.getParent().getParent() != null) {
                    dirs.add(p.getParent().getParent().resolve("bin"));
                }
            }
        }
        dirs.add(Paths.get("opencv", "build", "bin").toAbsolutePath());
        dirs.add(Paths.get("..", "opencv", "build", "bin").toAbsolutePath());
 
        for (Path dir : dirs) {
            for (String dll : dllNames) {
                Path candidate = dir.resolve(dll).normalize();
                if (!Files.isRegularFile(candidate)) continue;
                try {
                    System.load(candidate.toString());
                    System.out.println("Loaded FFmpeg videoio plugin: " + candidate);
                    return;
                } catch (Throwable t) {
                    System.err.println("Could not load " + candidate + ": " + t.getMessage());
                }
            }
        }
        System.err.println("FFmpeg videoio plugin not loaded. MSMF will be used for .mov "
                + "and may fail RGB32. Copy opencv_videoio_ffmpeg490_64.dll next to "
                + "opencv_java490.dll or into opencv\\build\\bin on PATH.");
    }
 
    static VideoCapture openVideo(Path path) {
        String file = path.toAbsolutePath().toString();
        int[] apis = { Videoio.CAP_FFMPEG, Videoio.CAP_ANY, Videoio.CAP_MSMF };
        String[] labels = { "FFMPEG", "ANY", "MSMF" };
        for (int a = 0; a < apis.length; a++) {
            for (double convert : new double[]{ (apis[a] == Videoio.CAP_MSMF ? 0 : 1), 0, 1 }) {
                VideoCapture cap = tryOpen(file, apis[a], labels[a], convert);
                if (cap != null) {
                    return cap;
                }
            }
        }
        VideoCapture cap = new VideoCapture();
        cap.open(file);
        if (cap.isOpened()) {
            cap.set(Videoio.CAP_PROP_CONVERT_RGB, 0);
            if (probeFrame(cap)) {
                logBackend(file, cap, "default");
                return cap;
            }
        }
        cap.release();
        return null;
    }
 
    static VideoCapture tryOpen(String file, int api, String label, double convertRgb) {
        VideoCapture cap = new VideoCapture();
        MatOfInt params = new MatOfInt(Videoio.CAP_PROP_CONVERT_RGB, (int) convertRgb);
        try {
            boolean opened = cap.open(file, api, params);
            if (!opened || !cap.isOpened()) {
                cap.release();
                params.release();
                return null;
            }
        } catch (Exception e) {
            cap.release();
            params.release();
            return null;
        }
        params.release();
        cap.set(Videoio.CAP_PROP_CONVERT_RGB, convertRgb);
        if (!probeFrame(cap)) {
            cap.release();
            return null;
        }
        logBackend(file, cap, label + " convertRGB=" + (int) convertRgb);
        return cap;
    }
 
    static boolean probeFrame(VideoCapture cap) {
        Mat probe = new Mat();
        boolean ok = cap.read(probe) && !probe.empty();
        if (ok) {
            Mat bgr = new Mat();
            ok = toBgr(probe, bgr);
            bgr.release();
        }
        probe.release();
        if (!ok) {
            return false;
        }
        cap.set(Videoio.CAP_PROP_POS_FRAMES, 0);
        cap.set(Videoio.CAP_PROP_POS_MSEC, 0);
        return true;
    }
 
    static void logBackend(String file, VideoCapture cap, String requested) {
        String backend = requested;
        try {
            String named = cap.getBackendName();
            if (named != null && !named.isEmpty()) {
                backend = named + " / " + requested;
            }
        } catch (Exception ignored) {
        }
        System.out.println("Opened " + file + " [" + backend + "]");
    }
 
    static boolean toBgr(Mat src, Mat dst) {
        if (src == null || src.empty()) {
            return false;
        }
        int type = src.type();
        if (type == CvType.CV_8UC3) {
            src.copyTo(dst);
            return true;
        }
        if (type == CvType.CV_8UC4) {
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGRA2BGR);
            return !dst.empty();
        }
        if (type == CvType.CV_8UC2) {
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_YUV2BGR_YUY2);
            return !dst.empty();
        }
        if (src.channels() == 1) {
            int h = src.rows();
            int w = src.cols();
            if (h * 2 % 3 == 0) {
                int visH = h * 2 / 3;
                if (visH > 0 && visH * 3 / 2 == h) {
                    try {
                        Imgproc.cvtColor(src, dst, Imgproc.COLOR_YUV2BGR_NV12);
                        if (!dst.empty() && dst.channels() == 3) {
                            return true;
                        }
                    } catch (Exception ignored) {
                    }
                }
            }
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_GRAY2BGR);
            return !dst.empty();
        }
        src.copyTo(dst);
        return !dst.empty();
    }
 
    static boolean readBgr(VideoCapture cap, Mat bgr) {
        if (cap == null || !cap.isOpened()) {
            return false;
        }
        Mat raw = new Mat();
        if (!cap.read(raw) || raw.empty()) {
            raw.release();
            return false;
        }
        boolean ok = toBgr(raw, bgr);
        raw.release();
        return ok && bgr != null && !bgr.empty();
    }
 
    static boolean readOrLoop(VideoCapture[] caps, int i, Path path, Mat frame) {
        if (readBgr(caps[i], frame)) {
            return true;
        }
        caps[i].set(Videoio.CAP_PROP_POS_FRAMES, 0);
        caps[i].set(Videoio.CAP_PROP_POS_MSEC, 0);
        if (readBgr(caps[i], frame)) {
            System.out.println("Video " + i + " looped via seek");
            return true;
        }
        caps[i].release();
        caps[i] = openVideo(path);
        if (caps[i] != null && readBgr(caps[i], frame)) {
            System.out.println("Video " + i + " reopened after loop");
            return true;
        }
        System.err.println("Warning: Could not read frame from video " + i);
        return false;
    }
 
    static byte[] encodeJpeg(Mat frame) {
        MatOfByte buf    = new MatOfByte();
        MatOfInt  params = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 88);
        Imgcodecs.imencode(".jpg", frame, buf, params);
        return buf.toArray();
    }
 
    static void writeFrame(OutputStream out, byte[] jpeg) throws IOException {
        String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                      + jpeg.length + "\r\n\r\n";
        out.write(header.getBytes("UTF-8"));
        out.write(jpeg);
        out.write("\r\n".getBytes("UTF-8"));
        out.flush();
    }
}