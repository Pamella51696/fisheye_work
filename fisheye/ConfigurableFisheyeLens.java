package fisheye;

/**
 * Single implementation for all supported forward projections; coefficients come from
 * {@link KannalaBrandtIntrinsics} plus optional Mei / double-sphere fields on the profile.
 */
public final class ConfigurableFisheyeLens implements FisheyeLensModel {

    private final KannalaBrandtIntrinsics intrinsics;
    private final FisheyeProjection projection;
    private final double xi;
    private final double alpha;

    public ConfigurableFisheyeLens(KannalaBrandtIntrinsics intrinsics,
                                   FisheyeProjection projection,
                                   double xi, double alpha) {
        this.intrinsics = intrinsics;
        this.projection = projection;
        this.xi = xi;
        this.alpha = alpha;
    }

    @Override
    public KannalaBrandtIntrinsics intrinsics() {
        return intrinsics;
    }

    @Override
    public FisheyeProjection projection() {
        return projection;
    }

    @Override
    public void rayToPixel(double x, double y, double z, double[] outUv) {
        switch (projection) {
            case KANNALA_BRANDT:
                intrinsics.rayToPixel(x, y, z, outUv);
                return;
            case MEI:
                meiRayToPixel(x, y, z, outUv);
                return;
            case DOUBLE_SPHERE:
                doubleSphereRayToPixel(x, y, z, outUv);
                return;
            default:
                idealRayToPixel(x, y, z, outUv);
        }
    }

    private void idealRayToPixel(double x, double y, double z, double[] outUv) {
        double r3 = Math.hypot(x, y);
        double theta = Math.atan2(r3, z);
        double rd;
        switch (projection) {
            case EQUISOLID:
                rd = 2.0 * Math.sin(theta * 0.5);
                break;
            case STEREOGRAPHIC:
                rd = 2.0 * Math.tan(theta * 0.5);
                break;
            case EQUIDISTANT:
            default:
                rd = theta;
                break;
        }
        double scale = r3 > 1e-10 ? rd / r3 : 0.0;
        outUv[0] = intrinsics.fx * x * scale + intrinsics.cx;
        outUv[1] = intrinsics.fy * y * scale + intrinsics.cy;
    }

    /** Unified / Mei model: mirror parameter ξ. */
    private void meiRayToPixel(double x, double y, double z, double[] outUv) {
        double r2 = x * x + y * y;
        double mz = z + xi * Math.sqrt(r2 + z * z);
        if (mz <= 1e-8) {
            outUv[0] = -1;
            outUv[1] = -1;
            return;
        }
        outUv[0] = intrinsics.fx * x / mz + intrinsics.cx;
        outUv[1] = intrinsics.fy * y / mz + intrinsics.cy;
    }

    /** Double-sphere model (Basalt / Kalibr). */
    private void doubleSphereRayToPixel(double x, double y, double z, double[] outUv) {
        double d1 = Math.sqrt(x * x + y * y + z * z);
        double k = xi * d1 + z;
        double d2 = Math.sqrt(x * x + y * y + k * k);
        double denom = alpha * d2 + (1.0 - alpha) * k;
        if (denom <= 1e-8) {
            outUv[0] = -1;
            outUv[1] = -1;
            return;
        }
        outUv[0] = intrinsics.fx * x / denom + intrinsics.cx;
        outUv[1] = intrinsics.fy * y / denom + intrinsics.cy;
    }
}
