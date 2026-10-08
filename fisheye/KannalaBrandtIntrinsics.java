package fisheye;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * OpenCV-compatible Kannala–Brandt fisheye intrinsics (equidistant + radial polynomial in θ).
 * <p>
 * Distorted normalized radius: {@code r_d = θ · (1 + k₁θ² + k₂θ⁴ + k₃θ⁶ + k₄θ⁸)} with
 * {@code θ = atan2(√(x²+y²), z)} on the unit ray.
 */
public final class KannalaBrandtIntrinsics {

    public final double fx;
    public final double fy;
    public final double cx;
    public final double cy;
    public final double k1;
    public final double k2;
    public final double k3;
    public final double k4;

    public KannalaBrandtIntrinsics(double fx, double fy, double cx, double cy,
                                   double k1, double k2, double k3, double k4) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.k1 = k1;
        this.k2 = k2;
        this.k3 = k3;
        this.k4 = k4;
    }

    /** Equidistant focal from full-frame incidence at the image circle (edge half-FOV). */
    public static KannalaBrandtIntrinsics fromFov(int width, int height, double fovDeg,
                                                  double cxFrac, double cyFrac) {
        double half = Math.toRadians(fovDeg) / 2.0;
        double f = (Math.min(width, height) / 2.0) / half;
        return new KannalaBrandtIntrinsics(
                f, f, width * cxFrac, height * cyFrac, 0, 0, 0, 0);
    }

    /**
     * Per-camera JSON or key=value lines: fx, fy, cx, cy, k1..k4 (optional fov_deg ignored if fx set).
     */
    public static KannalaBrandtIntrinsics load(Path path, int width, int height,
                                               double defaultFovDeg,
                                               double cxFrac, double cyFrac) throws java.io.IOException {
        if (path == null || !Files.isRegularFile(path)) {
            return fromFov(width, height, defaultFovDeg, cxFrac, cyFrac);
        }
        String text = Files.readString(path);
        double fx = num(text, "fx", Double.NaN);
        double fy = num(text, "fy", Double.NaN);
        double cx = num(text, "cx", Double.NaN);
        double cy = num(text, "cy", Double.NaN);
        double k1 = num(text, "k1", 0);
        double k2 = num(text, "k2", 0);
        double k3 = num(text, "k3", 0);
        double k4 = num(text, "k4", 0);
        double fov = num(text, "fov_deg", defaultFovDeg);
        if (Double.isNaN(fx) || Double.isNaN(fy)) {
            KannalaBrandtIntrinsics def = fromFov(width, height, fov, cxFrac, cyFrac);
            fx = def.fx;
            fy = def.fy;
        }
        if (Double.isNaN(cx)) {
            cx = width * cxFrac;
        }
        if (Double.isNaN(cy)) {
            cy = height * cyFrac;
        }
        return new KannalaBrandtIntrinsics(fx, fy, cx, cy, k1, k2, k3, k4);
    }

    static double readNumber(String text, String key, double fallback) {
        Pattern p = Pattern.compile(
                "\"?" + key + "\"?\\s*[:=]\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)",
                Pattern.CASE_INSENSITIVE);
        Matcher m = p.matcher(text);
        if (m.find()) {
            return Double.parseDouble(m.group(1));
        }
        return fallback;
    }

    static String readString(String text, String key) {
        Pattern p = Pattern.compile(
                "\"?" + key + "\"?\\s*[:=]\\s*\"([^\"]+)\"",
                Pattern.CASE_INSENSITIVE);
        Matcher m = p.matcher(text);
        if (m.find()) {
            return m.group(1);
        }
        Pattern bare = Pattern.compile(
                "\"?" + key + "\"?\\s*[:=]\\s*([A-Za-z_]+)",
                Pattern.CASE_INSENSITIVE);
        Matcher m2 = bare.matcher(text);
        return m2.find() ? m2.group(1) : null;
    }

    private static double num(String text, String key, double fallback) {
        return readNumber(text, key, fallback);
    }

    /** θ with KB radial factor (OpenCV fisheye θ_d). */
    public double thetaDistorted(double theta) {
        double t2 = theta * theta;
        double poly = 1.0 + k1 * t2 + k2 * t2 * t2 + k3 * t2 * t2 * t2 + k4 * t2 * t2 * t2 * t2;
        return theta * poly;
    }

    /**
     * Unit ray in camera frame (X right, Y down, Z forward) → distorted pixel.
     */
    public void rayToPixel(double x, double y, double z, double[] outUv) {
        double r = Math.hypot(x, y);
        double theta = Math.atan2(r, z);
        double thetaD = thetaDistorted(theta);
        double s = r > 1e-10 ? thetaD / r : 0.0;
        outUv[0] = fx * x * s + cx;
        outUv[1] = fy * y * s + cy;
    }

    /**
     * Incidence from optical axis and azimuth in the image plane → distorted pixel
     * (matches legacy equidistant sampling when k₁…k₄ = 0).
     */
    public void incidenceToPixel(double incidence, double azimuth, double[] outUv) {
        double thetaD = thetaDistorted(incidence);
        double r = thetaD;
        outUv[0] = cx + fx * r * Math.cos(azimuth);
        outUv[1] = cy + fy * r * Math.sin(azimuth);
    }

    /**
     * Invert {@code r_d = θ_d(θ)} with Newton iterations; returns incidence θ for normalized radius r_d.
     */
    public double thetaFromNormalizedRadius(double rd) {
        if (rd <= 0) {
            return 0;
        }
        double theta = rd;
        for (int i = 0; i < 12; i++) {
            double t2 = theta * theta;
            double poly = 1.0 + k1 * t2 + k2 * t2 * t2 + k3 * t2 * t2 * t2 + k4 * t2 * t2 * t2 * t2;
            double fVal = theta * poly - rd;
            double dPoly = k1 + 2 * k2 * t2 + 3 * k3 * t2 * t2 + 4 * k4 * t2 * t2 * t2;
            double df = poly + theta * dPoly;
            if (Math.abs(df) < 1e-12) {
                break;
            }
            theta -= fVal / df;
        }
        return theta;
    }
}
