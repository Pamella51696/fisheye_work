package surround.analytics;

import output.CurbZoneType;

/** Latest curb analytics snapshot for Android / signal API. */
public final class CurbState {

    public enum Status {
        OK,
        NO_CURB,
        UNAVAILABLE
    }

    public final long timestampMs;
    public final Status status;
    public final boolean detected;
    public final String camera;
    public final double distanceMeters;
    public final CurbZoneType zone;
    public final double confidence;

    public CurbState(long timestampMs,
                     Status status,
                     boolean detected,
                     String camera,
                     double distanceMeters,
                     CurbZoneType zone,
                     double confidence) {
        this.timestampMs = timestampMs;
        this.status = status;
        this.detected = detected;
        this.camera = camera;
        this.distanceMeters = distanceMeters;
        this.zone = zone;
        this.confidence = confidence;
    }

    public static CurbState unavailable() {
        return new CurbState(System.currentTimeMillis(), Status.UNAVAILABLE,
                false, "", Double.NaN, CurbZoneType.UNKNOWN, 0.0);
    }

    public static CurbState noCurb(long timestampMs) {
        return new CurbState(timestampMs, Status.NO_CURB,
                false, "", Double.NaN, CurbZoneType.UNKNOWN, 0.0);
    }
}
