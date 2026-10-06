package geometry;

import surround.geometry.PoseMath;

/**
 * Image (rectilinear) → camera → vehicle coordinates.
 * Vehicle frame: X forward, Y right, Z up. Ground plane z = groundZ.
 */
public final class CoordinateTransform {

    private CoordinateTransform() {
    }

    /** Unit ray in OpenCV camera frame from a rectilinear pixel. */
    public static void rectilinearPixelToCameraRay(double u, double v,
                                                   double fx, double fy,
                                                   double cx, double cy,
                                                   double[] outRayCam) {
        double xn = (u + 0.5 - cx) / fx;
        double yn = (v + 0.5 - cy) / fy;
        double len = Math.sqrt(xn * xn + yn * yn + 1.0);
        outRayCam[0] = xn / len;
        outRayCam[1] = yn / len;
        outRayCam[2] = 1.0 / len;
    }

    /** Camera ray → vehicle unit ray. {@code R} is camera-to-vehicle (row-major 3×3). */
    public static void cameraRayToVehicle(double[] rayCam, double[] R, double[] outRayVeh) {
        outRayVeh[0] = R[0] * rayCam[0] + R[3] * rayCam[1] + R[6] * rayCam[2];
        outRayVeh[1] = R[1] * rayCam[0] + R[4] * rayCam[1] + R[7] * rayCam[2];
        outRayVeh[2] = R[2] * rayCam[0] + R[5] * rayCam[1] + R[8] * rayCam[2];
        double len = Math.hypot(outRayVeh[0], Math.hypot(outRayVeh[1], outRayVeh[2]));
        if (len > 1e-9) {
            outRayVeh[0] /= len;
            outRayVeh[1] /= len;
            outRayVeh[2] /= len;
        }
    }

    /**
     * Ground-range distance from the vehicle origin to where the ray hits z = groundZ.
     * Camera rig at (0,0,cameraHeightZ). Returns NaN if ray does not hit ground forward.
     */
    public static double groundRangeMeters(double u, double v,
                                         double fx, double fy, double cx, double cy,
                                         double[] cameraToVehicle,
                                         double cameraHeightZ, double groundZ) {
        double[] rayCam = new double[3];
        double[] rayVeh = new double[3];
        rectilinearPixelToCameraRay(u, v, fx, fy, cx, cy, rayCam);
        cameraRayToVehicle(rayCam, cameraToVehicle, rayVeh);
        double dz = groundZ - cameraHeightZ;
        if (rayVeh[2] >= -1e-6) {
            return Double.NaN;
        }
        double t = dz / rayVeh[2];
        if (t <= 0) {
            return Double.NaN;
        }
        double gx = t * rayVeh[0];
        double gy = t * rayVeh[1];
        return Math.hypot(gx, gy);
    }

    public static void groundPointMeters(double u, double v,
                                         double fx, double fy, double cx, double cy,
                                         double[] cameraToVehicle,
                                         double cameraHeightZ, double groundZ,
                                         double[] outXyz) {
        double[] rayCam = new double[3];
        double[] rayVeh = new double[3];
        rectilinearPixelToCameraRay(u, v, fx, fy, cx, cy, rayCam);
        cameraRayToVehicle(rayCam, cameraToVehicle, rayVeh);
        double dz = groundZ - cameraHeightZ;
        if (rayVeh[2] >= -1e-6) {
            outXyz[0] = Double.NaN;
            outXyz[1] = Double.NaN;
            outXyz[2] = Double.NaN;
            return;
        }
        double t = dz / rayVeh[2];
        outXyz[0] = t * rayVeh[0];
        outXyz[1] = t * rayVeh[1];
        outXyz[2] = groundZ;
    }

    public static double[] rotationCameraToVehicle(double yawDeg, double pitchDeg, double rollDeg) {
        return PoseMath.cameraToVehicle(yawDeg, pitchDeg, rollDeg);
    }
}
