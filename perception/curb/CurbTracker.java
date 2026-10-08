package perception.curb;

import configuration.CurbConfig;
import output.ImagePoint;
import output.TrackingStatus;

import java.util.ArrayList;
import java.util.List;

/** Temporal smoothing on curb polylines to reduce flicker. */
public final class CurbTracker {

    private final double alpha;
    private List<ImagePoint> previous = List.of();
    private TrackingStatus status = TrackingStatus.LOST;
    private int missFrames;

    public CurbTracker(CurbConfig config) {
        this.alpha = Math.max(0.05, Math.min(0.95, config.trackingSmoothing));
    }

    public List<ImagePoint> update(List<ImagePoint> measured, double confidence, double minConfidence) {
        if (measured == null || measured.isEmpty() || confidence < minConfidence) {
            missFrames++;
            if (missFrames <= 4 && !previous.isEmpty()) {
                status = TrackingStatus.PREDICTED;
                return previous;
            }
            status = TrackingStatus.LOST;
            previous = List.of();
            return List.of();
        }
        missFrames = 0;
        if (previous.isEmpty()) {
            previous = measured;
            status = TrackingStatus.ACQUIRED;
            return measured;
        }
        List<ImagePoint> smoothed = smooth(previous, measured, alpha);
        previous = smoothed;
        status = TrackingStatus.TRACKING;
        return smoothed;
    }

    public TrackingStatus status() {
        return status;
    }

    private static List<ImagePoint> smooth(List<ImagePoint> a, List<ImagePoint> b, double alpha) {
        int n = Math.min(a.size(), b.size());
        if (n == 0) {
            return b;
        }
        List<ImagePoint> out = new ArrayList<>(n);
        double beta = 1.0 - alpha;
        for (int i = 0; i < n; i++) {
            out.add(new ImagePoint(
                    alpha * a.get(i).x + beta * b.get(i).x,
                    alpha * a.get(i).y + beta * b.get(i).y));
        }
        if (b.size() > n) {
            for (int i = n; i < b.size(); i++) {
                out.add(b.get(i));
            }
        }
        return out;
    }
}
