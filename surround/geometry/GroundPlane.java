package surround.geometry;

/**
 * Vehicle frame: X forward, Y right, Z up. Ground is the horizontal plane {@code z = groundZ};
 * camera rig at {@code (0,0,cameraHeightZ)}.
 */
public final class GroundPlane {

    private GroundPlane() {
    }

    /**
     * Unit ray from the rig origin toward a point on the ground plane at vehicle yaw {@code theta}
     * and ground distance {@code groundRange}.
     */
    public static void rayTowardGroundPoint(double theta, double groundRange,
                                            double cameraHeightZ, double groundZ,
                                            double[] outUnitRay) {
        double gx = groundRange * Math.cos(theta);
        double gy = groundRange * Math.sin(theta);
        double dx = gx;
        double dy = gy;
        double dz = groundZ - cameraHeightZ;
        double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-9) {
            outUnitRay[0] = 0;
            outUnitRay[1] = 0;
            outUnitRay[2] = -1;
            return;
        }
        outUnitRay[0] = dx / len;
        outUnitRay[1] = dy / len;
        outUnitRay[2] = dz / len;
    }

    /** Spherical vehicle ray (existing panel convention). */
    public static void rayFromSpherical(double theta, double phi, double[] outUnitRay) {
        double cphi = Math.cos(phi);
        outUnitRay[0] = cphi * Math.cos(theta);
        outUnitRay[1] = cphi * Math.sin(theta);
        outUnitRay[2] = Math.sin(phi);
    }

    public static void lerp3(double[] a, double[] b, double t, double[] out) {
        out[0] = a[0] * (1.0 - t) + b[0] * t;
        out[1] = a[1] * (1.0 - t) + b[1] * t;
        out[2] = a[2] * (1.0 - t) + b[2] * t;
        double len = Math.sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        if (len > 1e-9) {
            out[0] /= len;
            out[1] /= len;
            out[2] /= len;
        }
    }
}
