package surround.analytics;

import output.CurbDetectionResult;
import output.CurbZoneType;
import output.FusedCurbResult;

/**
 * Reduces detector output rate for Android: zone changes, large distance deltas,
 * and periodic heartbeat while active.
 */
public final class CurbResultFilter {

    private final double distanceDeltaMeters;
    private final long heartbeatMs;

    private CurbState lastPublished = CurbState.unavailable();
    private long lastPublishMs;

    public CurbResultFilter(double distanceDeltaMeters, long heartbeatMs) {
        this.distanceDeltaMeters = distanceDeltaMeters;
        this.heartbeatMs = heartbeatMs;
    }

    public synchronized CurbState filter(FusedCurbResult fused) {
        CurbState candidate = toState(fused);
        long now = System.currentTimeMillis();
        if (shouldPublish(candidate, now)) {
            lastPublished = candidate;
            lastPublishMs = now;
        }
        return lastPublished;
    }

    public synchronized CurbState lastPublished() {
        return lastPublished;
    }

    private boolean shouldPublish(CurbState candidate, long now) {
        if (lastPublished.status == CurbState.Status.UNAVAILABLE) {
            return true;
        }
        if (candidate.status != lastPublished.status) {
            return true;
        }
        if (candidate.zone != lastPublished.zone) {
            return true;
        }
        if (candidate.detected != lastPublished.detected) {
            return true;
        }
        if (candidate.detected && lastPublished.detected) {
            if (Math.abs(candidate.distanceMeters - lastPublished.distanceMeters) >= distanceDeltaMeters) {
                return true;
            }
        }
        return now - lastPublishMs >= heartbeatMs;
    }

    private static CurbState toState(FusedCurbResult fused) {
        if (!fused.valid || !fused.curbDetected) {
            return CurbState.noCurb(fused.timestampMs);
        }
        CurbDetectionResult best = pickClosest(fused);
        CurbZoneType zone = CurbZoneType.UNKNOWN;
        if (best != null && best.zones != null && !best.zones.isEmpty()) {
            zone = best.zones.get(best.zones.size() - 1).type;
        }
        double dist = fused.distanceMinMeters;
        String cam = best != null ? best.cameraId : "";
        return new CurbState(fused.timestampMs, CurbState.Status.OK,
                true, cam, dist, zone, fused.confidence);
    }

    private static CurbDetectionResult pickClosest(FusedCurbResult fused) {
        CurbDetectionResult best = null;
        double min = Double.POSITIVE_INFINITY;
        for (CurbDetectionResult c : fused.perCamera) {
            if (!c.curbDetected || !c.valid) {
                continue;
            }
            if (c.distanceMinMeters < min) {
                min = c.distanceMinMeters;
                best = c;
            }
        }
        return best;
    }
}
