package output;

import java.util.Collections;
import java.util.List;

/** Per-camera curb perception output (no UI / overlay instructions). */
public final class CurbDetectionResult {
    public final long timestampMs;
    public final String cameraId;
    public final boolean curbDetected;
    public final double confidence;
    public final TrackingStatus trackingStatus;
    public final List<ImagePoint> curbPoints;
    public final double distanceMinMeters;
    public final double distanceMaxMeters;
    public final List<CurbZoneSegment> zones;
    public final boolean valid;

    public CurbDetectionResult(long timestampMs,
                               String cameraId,
                               boolean curbDetected,
                               double confidence,
                               TrackingStatus trackingStatus,
                               List<ImagePoint> curbPoints,
                               double distanceMinMeters,
                               double distanceMaxMeters,
                               List<CurbZoneSegment> zones,
                               boolean valid) {
        this.timestampMs = timestampMs;
        this.cameraId = cameraId;
        this.curbDetected = curbDetected;
        this.confidence = confidence;
        this.trackingStatus = trackingStatus;
        this.curbPoints = curbPoints == null ? List.of() : curbPoints;
        this.distanceMinMeters = distanceMinMeters;
        this.distanceMaxMeters = distanceMaxMeters;
        this.zones = zones == null ? List.of() : zones;
        this.valid = valid;
    }

    public static CurbDetectionResult noDetection(long timestampMs, String cameraId) {
        return new CurbDetectionResult(timestampMs, cameraId, false, 0.0,
                TrackingStatus.LOST, Collections.emptyList(),
                Double.NaN, Double.NaN, Collections.emptyList(), false);
    }
}
