package com.middleware.panorama.wp5;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import configuration.CurbConfig;
import output.CurbZoneType;
import perception.curb.CurbPerceptionPipeline;
import surround.analytics.CurbDetectionService;
import surround.analytics.CurbResultFilter;
import surround.analytics.CurbState;
import surround.runtime.CameraCaptureManager;
import surround.runtime.CameraIo;
import surround.runtime.FrameDistributor;
import surround.runtime.SurroundPipeline;

/**
 * Optional camera-derived curb feed into WP5 {@link SignalHub} (does not use distance sliders).
 * Enable with {@code -Dwp5.cameraCurb=true} and {@code config/rig.json} + {@code config/curb.json}.
 */
public final class Wp5CameraCurbBridge implements AutoCloseable {

    private final Wp5Service wp5;
    private final Wp5Config cfg;
    private final ScheduledExecutorService publisher;
    private final CurbDetectionService curbService;
    private final CameraCaptureManager captureManager;
    private final FrameDistributor distributor;
    private long curbSeq;

    private Wp5CameraCurbBridge(Wp5Service wp5,
                                Wp5Config cfg,
                                CurbDetectionService curbService,
                                CameraCaptureManager captureManager,
                                FrameDistributor distributor) {
        this.wp5 = wp5;
        this.cfg = cfg;
        this.curbService = curbService;
        this.captureManager = captureManager;
        this.distributor = distributor;
        this.publisher = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "wp5-camera-curb");
            t.setDaemon(true);
            return t;
        });
    }

    public static Wp5CameraCurbBridge start(Wp5Service wp5, Path clipFolder) throws IOException {
        if (!Boolean.getBoolean("wp5.cameraCurb")) {
            return null;
        }
        Path rig = Paths.get("config", "rig.json");
        Path curbJson = Paths.get("config", "curb.json");
        Path[] videos = KannalaClipDiscovery.discoverRoleClips(clipFolder);
        if (videos == null) {
            throw new IOException("camera curb: need left/front/right/rear clips in " + clipFolder);
        }
        for (int i = 0; i < videos.length; i++) {
            videos[i] = CameraIo.ensureDecodable(videos[i]);
        }
        Wp5Config cfg = Wp5Config.load();
        SurroundPipeline pipeline = SurroundPipeline.load(rig);
        CurbConfig curbConfig = CurbConfig.load(curbJson);
        FrameDistributor distributor = new FrameDistributor();
        CameraCaptureManager capture = new CameraCaptureManager(videos, distributor);
        capture.start();
        CurbPerceptionPipeline perception = new CurbPerceptionPipeline(curbConfig, pipeline);
        CurbResultFilter filter = new CurbResultFilter(0.08, 500);
        CurbDetectionService curbService = new CurbDetectionService(distributor, perception, filter);
        curbService.start();
        Wp5CameraCurbBridge bridge = new Wp5CameraCurbBridge(wp5, cfg, curbService, capture, distributor);
        bridge.publisher.scheduleAtFixedRate(bridge::publishCurb, 200, 200, TimeUnit.MILLISECONDS);
        System.out.println("[WP5] camera curb ON (perception → SSE curb topic; sliders ignored for zones)");
        return bridge;
    }

    private void publishCurb() {
        try {
            CurbState state = curbService.currentState();
            String json = toWp5CurbJson(state, cfg, ++curbSeq);
            wp5.publishCurbJson(json);
        } catch (Throwable t) {
            System.err.println("[WP5] camera curb publish: " + t.getMessage());
        }
    }

    static String toWp5CurbJson(CurbState state, Wp5Config cfg, long seq) {
        long ts = state.timestampMs > 0 ? state.timestampMs : System.currentTimeMillis();
        if (state.status == CurbState.Status.UNAVAILABLE) {
            return "{\"type\":\"curb\",\"v\":1,\"seq\":" + seq + ",\"ts\":" + ts
                    + ",\"perceptionSource\":\"CAMERA\",\"detectionStatus\":\"UNAVAILABLE\",\"zones\":[]}";
        }
        String zoneName = state.zone == null ? "NONE" : mapZone(state.zone);
        String color = zoneColor(cfg, state.zone);
        boolean stale = state.status != CurbState.Status.OK && state.status != CurbState.Status.NO_CURB;
        double dist = state.detected && !Double.isNaN(state.distanceMeters) ? state.distanceMeters : -1;
        String distJson = dist >= 0 ? Json.num(dist) : "null";
        String pos = mapCameraToSensorPos(state.camera);
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\"type\":\"curb\",\"v\":1,\"seq\":").append(seq).append(",\"ts\":").append(ts)
                .append(",\"perceptionSource\":\"CAMERA\",\"detectionStatus\":\"")
                .append(stale ? "STALE" : "OK")
                .append("\",\"thresholds\":{\"dangerM\":").append(Json.num(cfg.dangerM))
                .append(",\"warningM\":").append(Json.num(cfg.warningM))
                .append("},\"zones\":[");
        if (state.detected && pos != null) {
            sb.append("{\"id\":\"CAM\",\"pos\":").append(Json.str(pos))
                    .append(",\"kind\":\"CURB\",\"source\":\"CAMERA\",\"distanceM\":").append(distJson)
                    .append(",\"zone\":").append(Json.str(zoneName))
                    .append(",\"color\":").append(Json.str(color))
                    .append(",\"confidence\":").append(Json.num(state.confidence))
                    .append('}');
        }
        sb.append("],\"summary\":{\"worstZone\":").append(Json.str(zoneName))
                .append(",\"nearestDistanceM\":").append(distJson)
                .append("}}");
        return sb.toString();
    }

    private static String mapZone(CurbZoneType z) {
        switch (z) {
            case RED: return "DANGER";
            case YELLOW: return "WARNING";
            case GREEN: return "SAFE";
            default: return "NONE";
        }
    }

    private static String zoneColor(Wp5Config cfg, CurbZoneType z) {
        if (z == CurbZoneType.RED) return cfg.color(Zone.DANGER);
        if (z == CurbZoneType.YELLOW) return cfg.color(Zone.WARNING);
        if (z == CurbZoneType.GREEN) return cfg.color(Zone.SAFE);
        return "#9E9E9E";
    }

    private static String mapCameraToSensorPos(String camera) {
        if (camera == null) return null;
        switch (camera.toLowerCase()) {
            case "left": return "LEFT_REAR";
            case "right": return "RIGHT_REAR";
            case "front": return "FRONT_CENTER";
            case "rear": return "REAR_CENTER";
            default: return null;
        }
    }

    @Override
    public void close() {
        publisher.shutdownNow();
        curbService.stop();
        captureManager.stop();
    }

    /** Discover left/front/right/rear clips (rig order), not panorama panel order. */
    static final class KannalaClipDiscovery {
        private static final String[] ROLES = {"left", "front", "right", "rear"};

        static Path[] discoverRoleClips(Path folder) {
            Path[] found = new Path[4];
            for (int i = 0; i < ROLES.length; i++) {
                found[i] = findClip(folder, ROLES[i]);
                if (found[i] == null) {
                    return null;
                }
            }
            return found;
        }

        static Path findClip(Path folder, String role) {
            String[] names = {role + "_1.mp4", role + ".mp4", role + "_1.MP4", role + ".MP4"};
            for (String n : names) {
                Path p = folder.resolve(n);
                if (java.nio.file.Files.isRegularFile(p)) {
                    return p;
                }
            }
            return null;
        }
    }
}
