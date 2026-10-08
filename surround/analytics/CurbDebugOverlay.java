package surround.analytics;

import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import output.CurbDetectionResult;
import output.CurbZoneSegment;
import output.CurbZoneType;
import output.ImagePoint;

/** Dev-only drawing on rectilinear frames (Android does real overlay in production). */
public final class CurbDebugOverlay {

    private CurbDebugOverlay() {
    }

    public static void draw(Mat bgr, CurbDetectionResult cam) {
        if (bgr == null || bgr.empty() || cam == null) {
            return;
        }
        String header;
        if (!cam.curbDetected || !cam.valid) {
            header = "CURB: not detected";
            Imgproc.putText(bgr, header, new Point(12, 28),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(180, 180, 180), 2);
            return;
        }
        header = String.format("CURB %s  %.2fm  conf=%.2f  %s",
                cam.cameraId, cam.distanceMinMeters, cam.confidence, cam.trackingStatus);
        Imgproc.putText(bgr, header, new Point(12, 28),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.55, new Scalar(255, 255, 255), 2);

        if (cam.zones != null) {
            for (CurbZoneSegment seg : cam.zones) {
                drawPolyline(bgr, seg.points, colorFor(seg.type), 3);
            }
        } else {
            drawPolyline(bgr, cam.curbPoints, new Scalar(0, 255, 255), 3);
        }
    }

    private static void drawPolyline(Mat bgr, List<ImagePoint> points, Scalar color, int thickness) {
        if (points == null || points.size() < 2) {
            return;
        }
        for (int i = 1; i < points.size(); i++) {
            ImagePoint a = points.get(i - 1);
            ImagePoint b = points.get(i);
            Imgproc.line(bgr, new Point(a.x, a.y), new Point(b.x, b.y), color, thickness);
        }
        for (ImagePoint p : points) {
            Imgproc.circle(bgr, new Point(p.x, p.y), 4, color, -1);
        }
    }

    private static Scalar colorFor(CurbZoneType type) {
        if (type == null) {
            return new Scalar(200, 200, 200);
        }
        switch (type) {
            case GREEN:
                return new Scalar(0, 220, 0);
            case YELLOW:
                return new Scalar(0, 220, 255);
            case RED:
                return new Scalar(0, 0, 255);
            default:
                return new Scalar(255, 180, 0);
        }
    }
}
