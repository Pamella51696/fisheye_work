package com.middleware.panorama.wp5;

import com.middleware.panorama.StitchingCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * WP-5 / Package 2 — curb zones (by default ONLY sensors with kind=CURB produce zones; -Dwp5.curbOnly=false = all sensors).
 *
 * Input : distance readings from ultrasonic / radar / lidar (per sensor).
 * Output: per-sensor RED / YELLOW / GREEN zone (with hysteresis, stale handling)
 *         + where that zone sits on the stitched panorama (pixel and 0..1 coordinates)
 *         + left/right/nearest summary for "context-aware emphasis" on Android.
 */
public final class CurbModel {

    /** Where each sensor position sits inside which camera strip: {cameraName, xFrom, xTo} (0..1). */
    static final Map<String, Object[]> DEFAULT_PANO_MAP = new LinkedHashMap<>();
    static {
        DEFAULT_PANO_MAP.put("FRONT_LEFT",   new Object[]{"front", 0.00, 0.34});
        DEFAULT_PANO_MAP.put("FRONT_CENTER", new Object[]{"front", 0.33, 0.67});
        DEFAULT_PANO_MAP.put("FRONT_RIGHT",  new Object[]{"front", 0.66, 1.00});
        // rear camera looks backwards -> vehicle-left appears on the image RIGHT
        DEFAULT_PANO_MAP.put("REAR_LEFT",    new Object[]{"rear",  0.66, 1.00});
        DEFAULT_PANO_MAP.put("REAR_CENTER",  new Object[]{"rear",  0.33, 0.67});
        DEFAULT_PANO_MAP.put("REAR_RIGHT",   new Object[]{"rear",  0.00, 0.34});
        // left camera looks out to the left -> vehicle-front appears on image RIGHT
        DEFAULT_PANO_MAP.put("LEFT_FRONT",   new Object[]{"left",  0.66, 1.00});
        DEFAULT_PANO_MAP.put("LEFT_REAR",    new Object[]{"left",  0.00, 0.34});
        // right camera looks out to the right -> vehicle-front appears on image LEFT
        DEFAULT_PANO_MAP.put("RIGHT_FRONT",  new Object[]{"right", 0.00, 0.34});
        DEFAULT_PANO_MAP.put("RIGHT_REAR",   new Object[]{"right", 0.66, 1.00});
    }

    private static final class Sensor {
        String id, pos, kind, source;
        double distanceM;       // <0 or >=maxRange => "clear"
        double heightM = Double.NaN;
        long receivedMs;
        Zone zone = Zone.NONE;  // remembered for hysteresis
    }

    /** Result of one evaluation. */
    public static final class CurbState {
        public final String json;
        public final String signature; // changes only when something visible changed
        public final Zone worst;
        CurbState(String json, String signature, Zone worst) {
            this.json = json; this.signature = signature; this.worst = worst;
        }
    }

    private final Wp5Config cfg;
    private final String[] cameraNames;
    private final int camW, camH, overlap;
    private final Map<String, Sensor> sensors = new LinkedHashMap<>();
    private final Map<String, Object[]> panoMap = new LinkedHashMap<>(DEFAULT_PANO_MAP);

    public CurbModel(Wp5Config cfg, String[] cameraNames, int camW, int camH, int overlapPx) {
        this.cfg = cfg; this.cameraNames = cameraNames;
        this.camW = camW; this.camH = camH;
        this.overlap = StitchingCore.clampOverlap(overlapPx, camW);
        panoMap.putAll(cfg.profile.panoOverrides);   // profile may change/add sensor positions
    }

    // ------------------------------------------------------------------ input

    private static final Pattern OBJ = Pattern.compile("\\{[^{}]*\\}");

    /** Parses {"sensors":[{...},{...}]} and stores the latest readings. Returns #accepted. */
    public synchronized int ingestJson(String body, long nowMs) {
        int ok = 0;
        Matcher m = OBJ.matcher(body.substring(Math.max(0, body.indexOf('['))));
        while (m.find()) {
            String o = m.group();
            String pos = Json.getStr(o, "pos", "").toUpperCase(Locale.ROOT);
            if (!panoMap.containsKey(pos)) continue;       // unknown position -> ignore
            String id = Json.getStr(o, "id", pos);
            String kind = Json.getStr(o, "kind", "UNKNOWN").toUpperCase(Locale.ROOT);
            if (cfg.curbOnly && !"CURB".equals(kind)) {      // ZONES ARE BASED ON CURB SENSORS ONLY
                sensors.remove(id);                          // obstacle / unknown sensor: no zone, not in summary
                continue;
            }
            double d = Json.getNum(o, "distanceM", -1);
            Sensor s = sensors.computeIfAbsent(id, k -> new Sensor());
            s.id = id; s.pos = pos;
            s.kind = kind;
            s.source = Json.getStr(o, "source", "UNKNOWN").toUpperCase(Locale.ROOT);
            s.distanceM = d;
            s.heightM = Json.getNum(o, "heightM", Double.NaN);
            s.receivedMs = nowMs;
            ok++;
        }
        return ok;
    }

    // ------------------------------------------------------------ classification

    /**
     * Pure function with hysteresis. Getting WORSE happens immediately at the threshold;
     * getting BETTER needs an extra hysteresisM so the colour does not flicker.
     */
    Zone classify(double d, Zone previous) {
        if (d < 0 || d >= cfg.maxRangeM) return Zone.SAFE;          // nothing in range
        Zone raw = d <= cfg.dangerM ? Zone.DANGER : (d <= cfg.warningM ? Zone.WARNING : Zone.SAFE);
        if (previous == Zone.NONE || raw.level >= previous.level) return raw;
        // improving: require margin beyond the threshold we are leaving
        double leaving = previous == Zone.DANGER ? cfg.dangerM : cfg.warningM;
        return d > leaving + cfg.hysteresisM ? raw : previous;
    }

    // ------------------------------------------------------------------ output

    public synchronized CurbState snapshot(long seq, long nowMs) {
        int n = cameraNames.length;
        int panoW = StitchingCore.panoramaWidth(n, camW, overlap);

        StringBuilder sb = new StringBuilder(1024);
        StringBuilder sig = new StringBuilder();
        Zone worst = Zone.NONE, wLeft = Zone.NONE, wRight = Zone.NONE;
        double nearest = Double.MAX_VALUE; String nearestId = null;

        sb.append("{\"type\":\"curb\",\"v\":1,\"seq\":").append(seq).append(",\"ts\":").append(nowMs)
          .append(",\"panorama\":{\"width\":").append(panoW).append(",\"height\":").append(camH).append('}')
          .append(",\"thresholds\":{\"dangerM\":").append(Json.num(cfg.dangerM))
          .append(",\"warningM\":").append(Json.num(cfg.warningM)).append('}')
          .append(",\"zones\":[");

        boolean first = true;
        for (Sensor s : sensors.values()) {
            boolean stale = nowMs - s.receivedMs > cfg.staleMs;
            Zone z = stale ? Zone.NONE : classify(s.distanceM, s.zone);
            s.zone = z;

            boolean clear = s.distanceM < 0 || s.distanceM >= cfg.maxRangeM;
            Object[] map = panoMap.get(s.pos);
            int camIdx = indexOf((String) map[0]);
            int x = 0, w = 0;
            if (camIdx >= 0) {
                x = StitchingCore.cameraX(camIdx, camW, overlap) + (int) Math.round(camW * (double) map[1]);
                w = (int) Math.round(camW * ((double) map[2] - (double) map[1]));
                if (x + w > panoW) w = panoW - x;
            }

            if (!first) sb.append(','); first = false;
            sb.append("{\"id\":").append(Json.str(s.id))
              .append(",\"pos\":").append(Json.str(s.pos))
              .append(",\"kind\":").append(Json.str(s.kind))
              .append(",\"source\":").append(Json.str(s.source))
              .append(",\"distanceM\":").append(clear || stale ? "null" : Json.num(s.distanceM))
              .append(",\"heightM\":").append(Double.isNaN(s.heightM) ? "null" : Json.num(s.heightM))
              .append(",\"zone\":").append(Json.str(z.name()))
              .append(",\"level\":").append(z.level)
              .append(",\"color\":").append(Json.str(cfg.color(z)))
              .append(",\"camera\":").append(Json.str((String) map[0]))
              .append(",\"pano\":{\"x\":").append(x).append(",\"w\":").append(w)
              .append(",\"xNorm\":").append(Json.num(panoW == 0 ? 0 : (double) x / panoW))
              .append(",\"wNorm\":").append(Json.num(panoW == 0 ? 0 : (double) w / panoW)).append("}}");

            sig.append(s.id).append(':').append(z.level).append(':')
               .append(clear || stale ? "-" : Long.toString(Math.round(s.distanceM * 50))).append(';'); // 2 cm steps

            worst = Zone.worst(worst, z);
            if (s.pos.contains("LEFT"))  wLeft  = Zone.worst(wLeft, z);
            if (s.pos.contains("RIGHT")) wRight = Zone.worst(wRight, z);
            if (!clear && !stale && s.distanceM < nearest) { nearest = s.distanceM; nearestId = s.id; }
        }
        sb.append("],\"summary\":{\"worstZone\":").append(Json.str(worst.name()))
          .append(",\"worstColor\":").append(Json.str(cfg.color(worst)))
          .append(",\"nearestDistanceM\":").append(nearestId == null ? "null" : Json.num(nearest))
          .append(",\"nearestSensorId\":").append(nearestId == null ? "null" : Json.str(nearestId))
          .append(",\"leftWorst\":").append(Json.str(wLeft.name()))
          .append(",\"rightWorst\":").append(Json.str(wRight.name()))
          .append("}}");
        return new CurbState(sb.toString(), sig.toString(), worst);
    }

    private int indexOf(String cam) {
        for (int i = 0; i < cameraNames.length; i++) if (cameraNames[i].equals(cam)) return i;
        return -1;
    }

    public synchronized boolean hasSensors() { return !sensors.isEmpty(); }
}
