package perception.curb;

import configuration.CurbConfig;
import output.CurbDetectionResult;
import output.CurbZoneSegment;
import output.FusedCurbResult;
import output.VehiclePoint;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Merges per-camera curb observations into a single vehicle-coordinate polyline.
 * Associates nearby ground points across cameras within {@code fusionAssociationMeters}.
 */
public final class CurbFusion {

    private final CurbConfig config;

    public CurbFusion(CurbConfig config) {
        this.config = config;
    }

    public FusedCurbResult fuse(long timestampMs, List<CurbDetectionResult> cameras,
                                List<List<VehiclePoint>> vehiclePolylines) {
        List<VehiclePoint> merged = new ArrayList<>();
        double confSum = 0;
        int confN = 0;
        boolean any = false;

        for (int i = 0; i < cameras.size(); i++) {
            CurbDetectionResult cam = cameras.get(i);
            if (!cam.curbDetected || !cam.valid) {
                continue;
            }
            any = true;
            confSum += cam.confidence;
            confN++;
            List<VehiclePoint> poly = vehiclePolylines.get(i);
            for (VehiclePoint p : poly) {
                if (!associate(merged, p)) {
                    merged.add(p);
                }
            }
        }

        merged.sort(Comparator.comparingDouble(p -> Math.atan2(p.y, p.x)));

        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (VehiclePoint p : merged) {
            double d = Math.hypot(p.x, p.y);
            if (d > config.maxDetectionRangeMeters) {
                continue;
            }
            min = Math.min(min, d);
            max = Math.max(max, d);
        }
        if (min == Double.POSITIVE_INFINITY) {
            min = Double.NaN;
            max = Double.NaN;
        }

        double confidence = confN == 0 ? 0.0 : confSum / confN;
        List<CurbZoneSegment> zones = fuseZones(cameras);
        boolean valid = any && confidence >= config.minConfidence && !merged.isEmpty();

        return new FusedCurbResult(timestampMs, any && !merged.isEmpty(), confidence,
                min, max, merged, zones, cameras, valid);
    }

    private boolean associate(List<VehiclePoint> merged, VehiclePoint p) {
        for (VehiclePoint q : merged) {
            if (Math.hypot(p.x - q.x, p.y - q.y) < config.fusionAssociationMeters) {
                return true;
            }
        }
        return false;
    }

    private static List<CurbZoneSegment> fuseZones(List<CurbDetectionResult> cameras) {
        List<CurbZoneSegment> all = new ArrayList<>();
        for (CurbDetectionResult c : cameras) {
            if (c.zones != null) {
                all.addAll(c.zones);
            }
        }
        return all;
    }
}
