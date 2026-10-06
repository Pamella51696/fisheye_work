package surround.geometry;

/** Camera-to-vehicle rotation (OpenCV camera frame → vehicle frame). */
public final class PoseMath {

    private PoseMath() {
    }

    public static double[] cameraToVehicle(double yawDeg, double pitchDeg, double rollDeg) {
        double[] rcv = {
            0, 0, 1,
            1, 0, 0,
            0,-1, 0
        };
        double[] rx = rotX(Math.toRadians(rollDeg));
        double[] ry = rotY(Math.toRadians(-pitchDeg));
        double[] rz = rotZ(Math.toRadians(yawDeg));
        return mul3(rz, mul3(ry, mul3(rx, rcv)));
    }

    static double[] rotX(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[] {1, 0, 0, 0, c,-s, 0, s, c};
    }

    static double[] rotY(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[] { c, 0, s, 0, 1, 0,-s, 0, c};
    }

    static double[] rotZ(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[] { c,-s, 0, s, c, 0, 0, 0, 1};
    }

    static double[] mul3(double[] a, double[] b) {
        double[] c = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                c[i * 3 + j] = a[i * 3] * b[j]
                        + a[i * 3 + 1] * b[3 + j]
                        + a[i * 3 + 2] * b[6 + j];
            }
        }
        return c;
    }
}
