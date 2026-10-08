package fisheye;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Remaps Kannala–Brandt fisheye frames to a rectilinear (pinhole) image.
 * <p>
 * For each destination pixel, build a pinhole ray and sample the source via the forward KB model.
 * Calibration is frozen per camera (maps rebuilt only when resolution changes).
 */
public final class FisheyeUndistortion {

    private final KannalaBrandtIntrinsics fisheye;
    private final double rectFx;
    private final double rectFy;
    private final double rectCx;
    private final double rectCy;

    private Mat map1;
    private Mat map2;
    private int cachedOutW = -1;
    private int cachedOutH = -1;

    public FisheyeUndistortion(KannalaBrandtIntrinsics fisheye) {
        this(fisheye, fisheye.fx, fisheye.fy, fisheye.cx, fisheye.cy);
    }

    /**
     * @param rectFx rectilinear focal lengths for the undistorted image (often same as fisheye f).
     */
    public FisheyeUndistortion(KannalaBrandtIntrinsics fisheye,
                               double rectFx, double rectFy, double rectCx, double rectCy) {
        this.fisheye = fisheye;
        this.rectFx = rectFx;
        this.rectFy = rectFy;
        this.rectCx = rectCx;
        this.rectCy = rectCy;
    }

    public KannalaBrandtIntrinsics fisheyeIntrinsics() {
        return fisheye;
    }

    public double rectFx() { return rectFx; }
    public double rectFy() { return rectFy; }
    public double rectCx() { return rectCx; }
    public double rectCy() { return rectCy; }

    public void undistort(Mat fisheyeBgr, Mat rectBgr) {
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
        if (map1 != null && outW == cachedOutW && outH == cachedOutH) {
            return;
        }
        Mat mapX = new Mat(outH, outW, CvType.CV_32FC1);
        Mat mapY = new Mat(outH, outW, CvType.CV_32FC1);
        float[] rowX = new float[outW];
        float[] rowY = new float[outW];
        double[] uv = new double[2];

        for (int v = 0; v < outH; v++) {
            for (int u = 0; u < outW; u++) {
                double xn = (u + 0.5 - rectCx) / rectFx;
                double yn = (v + 0.5 - rectCy) / rectFy;
                double norm = Math.sqrt(xn * xn + yn * yn + 1.0);
                fisheye.rayToPixel(xn / norm, yn / norm, 1.0 / norm, uv);
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
        cachedOutW = outW;
        cachedOutH = outH;
    }
}
