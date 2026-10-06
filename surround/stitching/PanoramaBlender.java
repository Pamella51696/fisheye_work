package surround.stitching;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import surround.calibration.PanoramaSettings;

public final class PanoramaBlender {

    private static final double INVALID_LUMA = 3.0;

    private final PanoramaSettings pano;

    public PanoramaBlender(PanoramaSettings pano) {
        this.pano = pano;
    }

    public Mat stitch(Mat[] panels) {
        int overlap = Math.max(16, Math.min(pano.panelWidth / 3, pano.overlapPx));
        int n = panels.length;
        int h = pano.panelHeight;
        int w = pano.panelWidth;
        int panoW = w + (n - 1) * (w - overlap);
        double[] gain = sequentialGains(panels, overlap);

        Mat accumColor = Mat.zeros(h, panoW, CvType.CV_32FC3);
        Mat accumWeight = Mat.zeros(h, panoW, CvType.CV_32FC1);
        Mat maxWeight = Mat.zeros(h, panoW, CvType.CV_32FC1);
        Mat maxColor = Mat.zeros(h, panoW, CvType.CV_32FC3);

        for (int i = 0; i < n; i++) {
            int xStart = i * (w - overlap);
            Mat weight = buildFeatherMask(h, w, overlap, i > 0, i < n - 1, 0.55, 0.55);
            Mat edgeW = edgeDistanceWeight(panels[i], pano.edgeFeatherPx);
            Core.multiply(weight, edgeW, weight);
            edgeW.release();

            Mat frameF = new Mat();
            panels[i].convertTo(frameF, CvType.CV_32FC3);
            if (Math.abs(gain[i] - 1.0) > 0.01) {
                Core.multiply(frameF, new Scalar(gain[i], gain[i], gain[i]), frameF);
            }
            Mat weight3 = new Mat();
            List<Mat> ch = new ArrayList<>();
            ch.add(weight);
            ch.add(weight);
            ch.add(weight);
            Core.merge(ch, weight3);
            Mat wFrame = new Mat();
            Core.multiply(frameF, weight3, wFrame);

            int xEnd = Math.min(xStart + w, panoW);
            int wActual = xEnd - xStart;
            Mat colorRoi = accumColor.submat(0, h, xStart, xEnd);
            Mat weightRoi = accumWeight.submat(0, h, xStart, xEnd);
            Core.add(colorRoi, wFrame.colRange(0, wActual), colorRoi);
            Core.add(weightRoi, weight.colRange(0, wActual), weightRoi);

            Mat maxColorRoi = maxColor.submat(0, h, xStart, xEnd);
            Mat maxWeightRoi = maxWeight.submat(0, h, xStart, xEnd);
            Mat greaterMask = new Mat();
            Core.compare(weight.colRange(0, wActual), maxWeightRoi, greaterMask, Core.CMP_GT);
            frameF.colRange(0, wActual).copyTo(maxColorRoi, greaterMask);
            weight.colRange(0, wActual).copyTo(maxWeightRoi, greaterMask);
            greaterMask.release();
            maxColorRoi.release();
            maxWeightRoi.release();
            colorRoi.release();
            weightRoi.release();
            frameF.release();
            weight.release();
            weight3.release();
            wFrame.release();
        }

        Mat safeW = new Mat();
        Core.max(accumWeight, new Scalar(1e-6), safeW);
        Mat safeW3 = new Mat();
        List<Mat> wch = new ArrayList<>();
        wch.add(safeW);
        wch.add(safeW);
        wch.add(safeW);
        Core.merge(wch, safeW3);
        Mat blended = new Mat();
        Core.divide(accumColor, safeW3, blended);

        Mat lowWeightMask = new Mat();
        Core.compare(accumWeight, new Scalar(0.02), lowWeightMask, Core.CMP_LT);
        maxColor.copyTo(blended, lowWeightMask);
        lowWeightMask.release();

        Mat result = new Mat();
        blended.convertTo(result, CvType.CV_8UC3);
        accumColor.release();
        accumWeight.release();
        maxWeight.release();
        maxColor.release();
        safeW.release();
        safeW3.release();
        blended.release();
        return result;
    }

    private static Mat buildFeatherMask(int h, int w, int overlap,
                                          boolean fadeLeft, boolean fadeRight,
                                          double inExp, double outExp) {
        Mat mask = new Mat(h, w, CvType.CV_32FC1, new Scalar(1.0));
        if (overlap <= 0) {
            return mask;
        }
        for (int x = 0; x < overlap; x++) {
            double cosine = 0.5 - 0.5 * Math.cos(Math.PI * x / overlap);
            if (fadeLeft) {
                float alpha = (float) Math.pow(cosine, inExp);
                Mat colL = mask.col(x);
                colL.setTo(new Scalar(alpha));
                colL.release();
            }
            if (fadeRight) {
                float alpha = (float) Math.pow(cosine, outExp);
                Mat colR = mask.col(w - 1 - x);
                colR.setTo(new Scalar(alpha));
                colR.release();
            }
        }
        return mask;
    }

    private static Mat edgeDistanceWeight(Mat bgr, int radius) {
        Mat gray = new Mat();
        Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
        Mat mask = new Mat();
        Imgproc.threshold(gray, mask, INVALID_LUMA, 255, Imgproc.THRESH_BINARY);
        Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(35, 35));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);
        closeKernel.release();
        Mat dist = new Mat();
        Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 3);
        Mat weight = new Mat();
        double scale = radius < 1 ? 1.0 : 1.0 / radius;
        dist.convertTo(weight, CvType.CV_32FC1, scale);
        Core.min(weight, new Scalar(1.0), weight);
        gray.release();
        mask.release();
        dist.release();
        return weight;
    }

    private static double[] sequentialGains(Mat[] frames, int overlap) {
        double[] gain = new double[frames.length];
        java.util.Arrays.fill(gain, 1.0);
        for (int i = 1; i < frames.length; i++) {
            double left = overlapMean(frames[i - 1], true, overlap);
            double right = overlapMean(frames[i], false, overlap);
            if (left < 8 || right < 8) {
                continue;
            }
            double g = left / right;
            if (g < 0.75) {
                g = 0.75;
            }
            if (g > 1.35) {
                g = 1.35;
            }
            gain[i] = gain[i - 1] * g;
            if (gain[i] < 0.75) {
                gain[i] = 0.75;
            }
            if (gain[i] > 1.35) {
                gain[i] = 1.35;
            }
        }
        return gain;
    }

    private static double overlapMean(Mat bgr, boolean rightEdge, int overlap) {
        int w = bgr.cols();
        int h = bgr.rows();
        int x0 = rightEdge ? w - overlap : 0;
        int x1 = rightEdge ? w : overlap;
        Mat roi = bgr.submat(0, h, x0, x1);
        Mat gray = new Mat();
        Imgproc.cvtColor(roi, gray, Imgproc.COLOR_BGR2GRAY);
        Mat mask = new Mat();
        Imgproc.threshold(gray, mask, INVALID_LUMA, 255, Imgproc.THRESH_BINARY);
        Scalar m = Core.mean(gray, mask);
        roi.release();
        gray.release();
        mask.release();
        return m.val[0];
    }
}
