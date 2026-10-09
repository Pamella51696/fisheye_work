package configuration;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

import surround.calibration.JsonUtil;

/** Configurable safety thresholds and geometry for curb perception. */
public final class CurbConfig {

    public final double redThresholdMeters;
    public final double yellowThresholdMeters;
    public final double minConfidence;
    public final double trackingSmoothing;
    public final int zoneHysteresisFrames;
    public final double cameraHeightZ;
    public final double groundPlaneZ;
    public final double maxDetectionRangeMeters;
    public final double fusionAssociationMeters;

    public CurbConfig(double redThresholdMeters,
                      double yellowThresholdMeters,
                      double minConfidence,
                      double trackingSmoothing,
                      int zoneHysteresisFrames,
                      double cameraHeightZ,
                      double groundPlaneZ,
                      double maxDetectionRangeMeters,
                      double fusionAssociationMeters) {
        this.redThresholdMeters = redThresholdMeters;
        this.yellowThresholdMeters = yellowThresholdMeters;
        this.minConfidence = minConfidence;
        this.trackingSmoothing = trackingSmoothing;
        this.zoneHysteresisFrames = zoneHysteresisFrames;
        this.cameraHeightZ = cameraHeightZ;
        this.groundPlaneZ = groundPlaneZ;
        this.maxDetectionRangeMeters = maxDetectionRangeMeters;
        this.fusionAssociationMeters = fusionAssociationMeters;
    }

    public static CurbConfig load(Path json) throws IOException {
        String text = Files.readString(json, StandardCharsets.UTF_8);
        Map<String, Object> root = JsonUtil.asObject(JsonUtil.parse(text));
        return new CurbConfig(
                JsonUtil.asDouble(root, "red_threshold_meters", 0.5),
                JsonUtil.asDouble(root, "yellow_threshold_meters", 1.0),
                JsonUtil.asDouble(root, "min_confidence", 0.5),
                JsonUtil.asDouble(root, "tracking_smoothing", 0.65),
                (int) JsonUtil.asDouble(root, "zone_hysteresis_frames", 3),
                JsonUtil.asDouble(root, "camera_height_z", 1.0),
                JsonUtil.asDouble(root, "ground_plane_z", 0.0),
                JsonUtil.asDouble(root, "max_detection_range_meters", 4.0),
                JsonUtil.asDouble(root, "fusion_association_meters", 0.35));
    }
}
