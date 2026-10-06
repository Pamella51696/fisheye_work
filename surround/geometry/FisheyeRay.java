package surround.geometry;

import surround.calibration.CameraIntrinsics;

/** Forward fisheye projection: unit camera ray → distorted pixel. */
public final class FisheyeRay {

    private FisheyeRay() {
    }

    public static void rayToPixel(double x, double y, double z, CameraIntrinsics intr, double[] outUv) {
        String model = intr.model == null ? "EQUIDISTANT" : intr.model.toUpperCase();
        double r = Math.hypot(x, y);
        double theta = Math.atan2(r, z);
        double thetaD;
        switch (model) {
            case "KANNALA_BRANDT":
            case "KB":
                thetaD = thetaDistorted(theta, intr);
                break;
            case "EQUISOLID":
                thetaD = 2.0 * Math.sin(theta * 0.5);
                break;
            case "STEROGRAPHIC":
                thetaD = 2.0 * Math.tan(theta * 0.5);
                break;
            case "EQUIDISTANT":
            default:
                thetaD = theta;
                break;
        }
        double s = r > 1e-10 ? thetaD / r : 0.0;
        outUv[0] = intr.fx * x * s + intr.cx;
        outUv[1] = intr.fy * y * s + intr.cy;
    }

    private static double thetaDistorted(double theta, CameraIntrinsics intr) {
        double t2 = theta * theta;
        double poly = 1.0 + intr.k1 * t2 + intr.k2 * t2 * t2
                + intr.k3 * t2 * t2 * t2 + intr.k4 * t2 * t2 * t2 * t2;
        return theta * poly;
    }
}
