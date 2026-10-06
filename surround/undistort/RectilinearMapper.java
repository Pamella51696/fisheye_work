package surround.undistort;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import surround.calibration.CameraIntrinsics;
import surround.geometry.FisheyeRay;

/**
 * Rectilinear debug view: pinhole ray per destination pixel → fisheye sample.
 * {@code balance} in [0,1] trades FOV vs crop (OpenCV balance analogue).
 */
public final class RectilinearMapper {

    private final CameraIntrinsics intrinsics;
    private final double balance;
    private final double outputFovDeg;
    private Mat map1;
    private Mat map2;
    private int cachedW = -1;
    private int cachedH = -1;

    public RectilinearMapper(CameraIntrinsics intrinsics, double balance, double outputFovDeg) {
        this.intrinsics = intrinsics;
        this.balance = balance;
        this.outputFovDeg = outputFovDeg;
    }

    public RectilinearSampling samplingForSize(int outW, int outH) {
        ensureMaps(outW, outH);
        double half = Math.toRadians(outputFovDeg) / 2.0;
        double fMin = (Math.min(outW, outH) / 2.0) / half;
        double fRect = fMin * (1.0 - balance) + intrinsics.fx * balance;
        return new RectilinearSampling(fRect, fRect, outW / 2.0, outH / 2.0);
    }

    public void rectify(Mat fisheyeBgr, Mat rectBgr) {
        if (fisheyeBgr == null || fisheyeBgr.empty()) {
            return;
        }
        int w = fisheyeBgr.cols();
        int h = fisheyeBgr.rows();
        ensureMaps(w, h);
        Imgproc.remap(fisheyeBgr, rectBgr, map1, map2, Imgproc.INTER_LINEAR,
                Core.BORDER_CONSTANT, new Scalar(0, 0, 0));
    }

    private void ensureMaps(int outW, int outH) {
        if (map1 != null && outW == cachedW && outH == cachedH) {
            return;
        }
        double half = Math.toRadians(outputFovDeg) / 2.0;
        double fMin = (Math.min(outW, outH) / 2.0) / half;
        double fMax = intrinsics.fx;
        double fRect = fMin * (1.0 - balance) + fMax * balance;
        double cx = outW / 2.0;
        double cy = outH / 2.0;
        double[] uv = new double[2];

        Mat mapX = new Mat(outH, outW, CvType.CV_32FC1);
        Mat mapY = new Mat(outH, outW, CvType.CV_32FC1);
        float[] rowX = new float[outW];
        float[] rowY = new float[outW];

        for (int v = 0; v < outH; v++) {
            for (int u = 0; u < outW; u++) {
                double xn = (u + 0.5 - cx) / fRect;
                double yn = (v + 0.5 - cy) / fRect;
                double norm = Math.sqrt(xn * xn + yn * yn + 1.0);
                FisheyeRay.rayToPixel(xn / norm, yn / norm, 1.0 / norm, intrinsics, uv);
                rowX[u] = (float) uv[0];
                rowY[u] = (float) uv[1];
            }
            mapX.put(v, 0, rowX);
            mapY.put(v, 0, rowY);
        }
        if (map1 == null) {
            map1 = new Mat();
        }
        if (map2 == null) {
            map2 = new Mat();
        }
        Imgproc.convertMaps(mapX, mapY, map1, map2, CvType.CV_16SC2, false);
        mapX.release();
        mapY.release();
        cachedW = outW;
        cachedH = outH;
    }
}
