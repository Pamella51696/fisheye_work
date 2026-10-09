package output;

import java.util.Collections;
import java.util.List;

/** Unified vehicle-coordinate curb model after multi-camera fusion. */
public final class FusedCurbResult {
    public final long timestampMs;
    public final boolean curbDetected;
    public final double confidence;
    public final double distanceMinMeters;
    public final double distanceMaxMeters;
    public final List<VehiclePoint> curbPointsVehicle;
    public final List<CurbZoneSegment> zones;
    public final List<CurbDetectionResult> perCamera;
    public final boolean valid;

    public FusedCurbResult(long timestampMs,
                           boolean curbDetected,
                           double confidence,
                           double distanceMinMeters,
                           double distanceMaxMeters,
                           List<VehiclePoint> curbPointsVehicle,
                           List<CurbZoneSegment> zones,
                           List<CurbDetectionResult> perCamera,
                           boolean valid) {
        this.timestampMs = timestampMs;
        this.curbDetected = curbDetected;
        this.confidence = confidence;
        this.distanceMinMeters = distanceMinMeters;
        this.distanceMaxMeters = distanceMaxMeters;
        this.curbPointsVehicle = curbPointsVehicle == null ? List.of() : curbPointsVehicle;
        this.zones = zones == null ? List.of() : zones;
        this.perCamera = perCamera == null ? List.of() : perCamera;
        this.valid = valid;
    }
}
