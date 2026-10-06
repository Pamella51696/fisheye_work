package surround.stitching;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import surround.calibration.CameraConfig;
import surround.calibration.PanoramaSettings;
import surround.geometry.FisheyeRay;
import surround.geometry.GroundPlane;
import surround.geometry.PoseMath;

/**
 * Precomputed map: panorama panel pixel → fisheye source (vehicle ray → camera ray → pixel).
 * Below the shared horizon, rays are anchored to a common ground plane when enabled.
 */
public final class PanoramaMapper {

    private final CameraConfig camera;
    private final PanoramaSettings pano;
    private final double[] R;
    private final double[] uv = new double[2];
    private final double[] rayV = new double[3];
    private final double[] rayG = new double[3];
    private final double[] rayS = new double[3];
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
        int blend = pano.groundBlendRows;

        Mat mapX = new Mat(ph, pw, CvType.CV_32FC1);
        Mat mapY = new Mat(ph, pw, CvType.CV_32FC1);
        float[] rowX = new float[pw];
        float[] rowY = new float[pw];

        for (int v = 0; v < ph; v++) {
            double phi = (horizonY - v) / ph * pitchSpan;
            for (int u = 0; u < pw; u++) {
                double theta = yaw0 + ((u + 0.5) / pw - 0.5) * yawSpan;

                if (pano.groundPlaneEnabled && v > horizonY - blend) {
                    double groundFrac = (v - horizonY) / Math.max(1.0, ph - horizonY);
                    if (groundFrac < 0) {
                        groundFrac = 0;
                    }
                    if (groundFrac > 1) {
                        groundFrac = 1;
                    }
                    double range = pano.groundDistanceNear
                            + groundFrac * (pano.groundDistanceFar - pano.groundDistanceNear);
                    GroundPlane.rayTowardGroundPoint(theta, range,
                            pano.cameraHeightZ, pano.groundPlaneZ, rayG);
                    GroundPlane.rayFromSpherical(theta, phi, rayS);
                    double t = 1.0;
                    if (v < horizonY + blend) {
                        t = (v - (horizonY - blend)) / Math.max(1.0, 2.0 * blend);
                        if (t < 0) {
                            t = 0;
                        }
                        if (t > 1) {
                            t = 1;
                        }
                    }
                    GroundPlane.lerp3(rayS, rayG, t, rayV);
                } else {
                    GroundPlane.rayFromSpherical(theta, phi, rayV);
                }

                double xc = R[0] * rayV[0] + R[3] * rayV[1] + R[6] * rayV[2];
                double yc = R[1] * rayV[0] + R[4] * rayV[1] + R[7] * rayV[2];
                double zc = R[2] * rayV[0] + R[5] * rayV[1] + R[8] * rayV[2];
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
