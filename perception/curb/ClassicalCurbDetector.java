package perception.curb;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import output.ImagePoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Classical road-edge detector: Sobel horizontal gradient + column scan in a curb-side ROI.
 * Suitable for simulation feeds; swap implementation for production segmentation.
 */
public final class ClassicalCurbDetector implements CurbDetector {

    private static final int SAMPLE_COLUMNS = 24;
    private static final double ROI_TOP_FRACTION = 0.35;
    private static final double MIN_GRADIENT = 12.0;

    @Override
    public List<ImagePoint> detect(Mat rectilinearBgr, String role, double[] outConfidence) {
        if (rectilinearBgr == null || rectilinearBgr.empty()) {
            outConfidence[0] = 0.0;
            return List.of();
        }
        int w = rectilinearBgr.cols();
        int h = rectilinearBgr.rows();
        int y0 = (int) (h * ROI_TOP_FRACTION);
        int y1 = h - 4;

        RoiSide side = roiSide(role);
        int xStart = side == RoiSide.LEFT ? 0 : (side == RoiSide.RIGHT ? w / 2 : w / 4);
        int xEnd = side == RoiSide.LEFT ? w / 2 : (side == RoiSide.RIGHT ? w : 3 * w / 4);

        Mat gray = new Mat();
        Imgproc.cvtColor(rectilinearBgr, gray, Imgproc.COLOR_BGR2GRAY);
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(gray, blurred, new org.opencv.core.Size(5, 5), 0);
        Mat gradY = new Mat();
        Imgproc.Sobel(blurred, gradY, CvType.CV_32F, 0, 1, 3);

        List<ImagePoint> points = new ArrayList<>();
        double totalStrength = 0.0;
        int hits = 0;

        for (int c = 0; c < SAMPLE_COLUMNS; c++) {
            double t = (c + 0.5) / SAMPLE_COLUMNS;
            int x = (int) (xStart + t * (xEnd - xStart));
            if (x < 1 || x >= w - 1) {
                continue;
            }
            double bestY = -1;
            double bestG = 0;
            for (int y = y1; y >= y0; y--) {
                double g = Math.abs(gradY.get(y, x)[0]);
                if (g > bestG) {
                    bestG = g;
                    bestY = y;
                }
            }
            if (bestY >= 0 && bestG >= MIN_GRADIENT) {
                points.add(new ImagePoint(x, bestY));
                totalStrength += bestG;
                hits++;
            }
        }

        gray.release();
        blurred.release();
        gradY.release();

        if (hits < SAMPLE_COLUMNS / 4) {
            outConfidence[0] = hits / (double) SAMPLE_COLUMNS * 0.4;
            return List.of();
        }
        double coverage = hits / (double) SAMPLE_COLUMNS;
        double strength = Math.min(1.0, totalStrength / (hits * 80.0));
        outConfidence[0] = Math.min(1.0, 0.35 * coverage + 0.65 * strength);
        return points;
    }

    private enum RoiSide { LEFT, RIGHT, BOTH }

    private static RoiSide roiSide(String role) {
        if (role == null) {
            return RoiSide.BOTH;
        }
        switch (role.toLowerCase()) {
            case "left":
                return RoiSide.RIGHT;
            case "right":
                return RoiSide.LEFT;
            case "front":
                return RoiSide.BOTH;
            case "rear":
                return RoiSide.BOTH;
            default:
                return RoiSide.BOTH;
        }
    }
}
