package perception.curb;

import configuration.CurbConfig;
import output.CurbZoneSegment;
import output.CurbZoneType;
import output.ImagePoint;

import java.util.ArrayList;
import java.util.List;

public final class CurbZoneClassifier {

    private final CurbConfig config;
    private final ZoneHysteresis hysteresis = new ZoneHysteresis();

    public CurbZoneClassifier(CurbConfig config) {
        this.config = config;
    }

    public List<CurbZoneSegment> classify(List<ImagePoint> points, double[] distancesMeters) {
        if (points.isEmpty() || distancesMeters.length != points.size()) {
            return List.of();
        }
        List<CurbZoneSegment> segments = new ArrayList<>();
        int start = 0;
        CurbZoneType current = classifyOne(distancesMeters[0]);
        for (int i = 1; i < points.size(); i++) {
            CurbZoneType z = classifyOne(distancesMeters[i]);
            if (z != current) {
                segments.add(buildSegment(current, points, distancesMeters, start, i - 1));
                start = i;
                current = z;
            }
        }
        segments.add(buildSegment(current, points, distancesMeters, start, points.size() - 1));
        return segments;
    }

    private CurbZoneType classifyOne(double distanceMeters) {
        CurbZoneType raw = CurbZoneType.fromDistance(
                distanceMeters, config.redThresholdMeters, config.yellowThresholdMeters);
        return hysteresis.filter(raw);
    }

    private static CurbZoneSegment buildSegment(CurbZoneType type,
                                                List<ImagePoint> points,
                                                double[] distances,
                                                int from, int to) {
        List<ImagePoint> segPts = new ArrayList<>();
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (int i = from; i <= to; i++) {
            segPts.add(points.get(i));
            double d = distances[i];
            if (!Double.isNaN(d)) {
                min = Math.min(min, d);
                max = Math.max(max, d);
            }
        }
        if (min == Double.POSITIVE_INFINITY) {
            min = Double.NaN;
            max = Double.NaN;
        }
        return new CurbZoneSegment(type, min, max, segPts);
    }

    private final class ZoneHysteresis {
        private CurbZoneType stable = CurbZoneType.UNKNOWN;
        private CurbZoneType candidate = CurbZoneType.UNKNOWN;
        private int candidateCount;

        CurbZoneType filter(CurbZoneType raw) {
            if (raw == stable) {
                candidate = raw;
                candidateCount = 0;
                return stable;
            }
            if (raw != candidate) {
                candidate = raw;
                candidateCount = 1;
            } else {
                candidateCount++;
            }
            if (candidateCount >= config.zoneHysteresisFrames) {
                stable = candidate;
            }
            return stable == CurbZoneType.UNKNOWN ? raw : stable;
        }
    }
}
