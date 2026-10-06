package surround.stitching;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import surround.calibration.CameraConfig;
import surround.calibration.PanoramaSettings;
import surround.geometry.FisheyeRay;
import surround.geometry.PoseMath;

/**
 * Precomputed map: panorama panel pixel → fisheye source sample (ray-based path).
 */
public final class PanoramaMapper {

    private final CameraConfig camera;
    private final PanoramaSettings pano;
    private final double[] R;
    private final double[] uv = new double[2];
    private Mat map1;
    private Mat map2;
    private int cachedW = -1;
    private int cachedH = -1;

    public PanoramaMapper(CameraConfig camera, PanoramaSettings pano) {
        this.camera = camera;
        this.pano = pano;
        this.R = PoseMath.cameraToVehicle(
                camera.extrinsics.effectiveYawDeg(),
                camera.extrinsics.effectivePitchDeg(),
                camera.extrinsics.rollDeg);
    }

    public void project(Mat fisheyeBgr, Mat panelBgr) {
        if (fisheyeBgr == null || fisheyeBgr.empty()) {
            return;
        }
        ensureMaps(fisheyeBgr.cols(), fisheyeBgr.rows());
        Imgproc.remap(fisheyeBgr, panelBgr, map1, map2, Imgproc.INTER_LINEAR,
                Core.BORDER_CONSTANT, new Scalar(0, 0, 0));
    }

    private void ensureMaps(int srcW, int srcH) {
        if (map1 != null && srcW == cachedW && srcH == cachedH) {
            return;
        }
        int pw = pano.panelWidth;
        int ph = pano.panelHeight;
        double maxInc = Math.toRadians(pano.maxIncidenceDeg);
        double yaw0 = Math.toRadians(camera.extrinsics.effectiveYawDeg());
        double yawSpan = Math.toRadians(pano.panelYawDeg);
        double pitchSpan = Math.toRadians(pano.panelPitchDeg);
        double horizonY = pano.horizonFraction * ph;

        Mat mapX = new Mat(ph, pw, CvType.CV_32FC1);
        Mat mapY = new Mat(ph, pw, CvType.CV_32FC1);
        float[] rowX = new float[pw];
        float[] rowY = new float[pw];

        for (int v = 0; v < ph; v++) {
            double phi = (horizonY - v) / ph * pitchSpan;
            double cphi = Math.cos(phi);
            double sphi = Math.sin(phi);
            for (int u = 0; u < pw; u++) {
                double theta = yaw0 + ((u + 0.5) / pw - 0.5) * yawSpan;
                double xv = cphi * Math.cos(theta);
                double yv = cphi * Math.sin(theta);
                double zv = sphi;
                double xc = R[0] * xv + R[3] * yv + R[6] * zv;
                double yc = R[1] * xv + R[4] * yv + R[7] * zv;
                double zc = R[2] * xv + R[5] * yv + R[8] * zv;
                if (zc <= 1e-4) {
                    rowX[u] = -1f;
                    rowY[u] = -1f;
                    continue;
                }
                double inc = Math.atan2(Math.hypot(xc, yc), zc);
                if (inc > maxInc) {
                    rowX[u] = -1f;
                    rowY[u] = -1f;
                    continue;
                }
                FisheyeRay.rayToPixel(xc, yc, zc, camera.intrinsics, uv);
                float su = (float) uv[0];
                float sv = (float) uv[1];
                if (su < 1 || sv < 1 || su >= srcW - 1 || sv >= srcH - 1) {
                    rowX[u] = -1f;
                    rowY[u] = -1f;
                } else {
                    rowX[u] = su;
                    rowY[u] = sv;
                }
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
        cachedW = srcW;
        cachedH = srcH;
    }
}
