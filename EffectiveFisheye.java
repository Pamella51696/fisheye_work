import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/**
 * Two-stage calibration for uncorrected fisheye feeds.
 *
 *   each camera alone: straight edges → K, D → 180° feed
 *   then ORB + RANSAC between neighboring feeds
 *
 * The output is a forward camera picture, not a top-down view.
 * Horizontal angle runs from −90° to +90°, so the frame is a 180° field.
 */
final class EffectiveFisheye {

    private static final double START_FOV_DEG = 200.0;
    /** Horizontal field of the corrected feed. The side edges are ±90°. */
    private static final double VIEW_HFOV_DEG = 180.0;
    /** Vertical field at the same degrees per pixel, in a 16:9 feed. */
    private static final double VIEW_VFOV_DEG = 101.25;
    private static final int VIEW_W = 1280;
    private static final int VIEW_H = 720;

    private EffectiveFisheye() {}

    static boolean isArgs(String[] args) {
        return has(args, "--correct-fisheye");
    }

    static boolean isSelfTestArgs(String[] args) {
        return has(args, "--correct-self-test");
    }

    private static boolean has(String[] args, String flag) {
        if (args == null) {
            return false;
        }
        for (String a : args) {
            if (flag.equals(a)) {
                return true;
            }
        }
        return false;
    }

    static int run(java.nio.file.Path folder) {
        System.out.println("STAGE 1  individual fisheye calibration");
        System.out.println("uncorrected feed → straight edges → K, D → 180° feed");
        Mat[] raw = VideoStreamingServer.representativeFrames(folder);
        if (raw == null) {
            return 2;
        }
        Lens[] lenses = new Lens[4];
        Mat[] corrected = new Mat[4];
        java.nio.file.Path outDir = folder.resolve("corrected");
        try {
            java.nio.file.Files.createDirectories(outDir);
        } catch (java.io.IOException e) {
            System.err.println("Could not create " + outDir);
            return 1;
        }
        for (int i = 0; i < 4; i++) {
            String role = VideoStreamingServer.CAM_ROLE[i];
            System.out.println("── " + role + " ──");
            Mat masked = maskFeed(raw[i]);
            lenses[i] = fit(masked, raw[i].cols(), raw[i].rows());
            masked.release();
            if (lenses[i] == null) {
                System.err.println("  " + role + ": not enough straight edges");
                continue;
            }
            Lens lens = lenses[i];
            System.out.printf(Locale.US,
                    "  K  fx=%.2f fy=%.2f cx=%.1f cy=%.1f%n  D  [%.4f %.4f %.4f %.4f]  FOV=%.1f°%n"
                            + "  line error %.5f → %.5f%n",
                    lens.fx, lens.fy, lens.cx, lens.cy,
                    lens.k1, lens.k2, lens.k3, lens.k4, lens.fov,
                    lens.before, lens.after);
            save(role, raw[i].cols(), raw[i].rows(), lens);
            corrected[i] = renderView(raw[i], lens);
            java.nio.file.Path file = outDir.resolve(role + ".jpg");
            Imgcodecs.imwrite(file.toString(), corrected[i]);
            System.out.println("  wrote " + file.toAbsolutePath());
            System.out.println("  180° feed: left edge −90°, right edge +90°");
        }

        System.out.println("STAGE 2  camera-to-camera alignment");
        System.out.println("180° feeds → ORB → RANSAC");
        double[] yaw = VideoStreamingServer.CAM_YAW_DEG.clone();
        double[] pitch = VideoStreamingServer.CAM_PITCH_DEG.clone();
        double[] roll = VideoStreamingServer.CAM_ROLL_DEG.clone();
        align(corrected, lenses, yaw, pitch, roll);
        for (int i = 0; i < 4; i++) {
            if (lenses[i] == null) {
                continue;
            }
            CalibrationManager.saveMountPose(
                    VideoStreamingServer.CAM_ROLE[i], yaw[i], pitch[i], roll[i]);
            System.out.printf(Locale.US, "  %s yaw=%.2f pitch=%.2f roll=%.2f%n",
                    VideoStreamingServer.CAM_ROLE[i], yaw[i], pitch[i], roll[i]);
        }
        for (int i = 0; i < 4; i++) {
            if (raw[i] != null) {
                raw[i].release();
            }
            if (corrected[i] != null) {
                corrected[i].release();
            }
        }
        return 0;
    }

    /** Hide the circular black border and the video-player bar. */
    private static Mat maskFeed(Mat src) {
        Mat dst = src.clone();
        int w = dst.cols();
        int h = dst.rows();
        double cx = w * 0.5;
        double cy = h * 0.5;
        double r = 0.47 * Math.min(w, h);
        int yCut = (int) (h * 0.90);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                double dx = x - cx;
                double dy = y - cy;
                if (y >= yCut || dx * dx + dy * dy > r * r) {
                    dst.put(y, x, 0, 0, 0);
                }
            }
        }
        return dst;
    }

    private static Lens fit(Mat masked, int w, int h) {
        List<double[][]> chains = LineFisheyeCalibrator.straightChains(masked);
        System.out.println("  straight features: " + chains.size());
        if (chains.size() < 4) {
            return null;
        }
        double minSide = Math.min(w, h);
        double f0 = VideoStreamingServer.fisheyeFocal(w, h, START_FOV_DEG);
        double[] p = {f0, f0, w * 0.5, h * 0.5, 0, 0, 0, 0};
        double before = cost(chains, p, w, h, minSide);
        double[] step = {f0 * 0.03, f0 * 0.03, w * 0.008, h * 0.008, 0.02, 0.012, 0.008, 0.006};
        p = nelder(p, step, 36, q -> cost(chains, q, w, h, minSide));
        double after = cost(chains, p, w, h, minSide);
        Lens lens = new Lens();
        lens.fx = p[0];
        lens.fy = p[1];
        lens.cx = p[2];
        lens.cy = p[3];
        lens.k1 = p[4];
        lens.k2 = p[5];
        lens.k3 = p[6];
        lens.k4 = p[7];
        lens.before = before;
        lens.after = after;
        lens.fov = fovOf(lens, minSide);
        lens.lines = chains.size();
        if (!(after < before * 0.98) || !Double.isFinite(after)) {
            System.out.println("  straightness did not improve; keeping the 200° start");
            lens.fx = lens.fy = f0;
            lens.cx = w * 0.5;
            lens.cy = h * 0.5;
            lens.k1 = lens.k2 = lens.k3 = lens.k4 = 0;
            lens.after = before;
            lens.fov = START_FOV_DEG;
        }
        return lens;
    }

    private static double fovOf(Lens lens, double minSide) {
        double[] ray = CalibrationManager.unprojectFisheye(
                lens.cx + 0.48 * minSide, lens.cy,
                new double[] {lens.fx, lens.fy, lens.cx, lens.cy},
                lens.k1, lens.k2, lens.k3, lens.k4);
        if (ray == null) {
            return Double.NaN;
        }
        double theta = Math.atan2(Math.hypot(ray[0], ray[1]), Math.max(ray[2], 1e-6));
        double edge = 0.50 * minSide;
        double sample = 0.48 * minSide;
        return Math.toDegrees(2.0 * theta * (edge / sample));
    }

    private static double cost(List<double[][]> chains, double[] p, int w, int h, double minSide) {
        if (!paramsOk(p, w, h, minSide)) {
            return 1e6;
        }
        double[] k = {p[0], p[1], p[2], p[3]};
        double sum = 0;
        double weight = 0;
        for (double[][] chain : chains) {
            double[] moment = new double[9];
            int n = 0;
            for (double[] px : chain) {
                double[] ray = CalibrationManager.unprojectFisheye(
                        px[0], px[1], k, p[4], p[5], p[6], p[7]);
                if (ray == null || ray[2] < 0.02) {
                    continue;
                }
                double norm = Math.hypot(ray[0], Math.hypot(ray[1], ray[2]));
                if (norm < 1e-8) {
                    continue;
                }
                ray[0] /= norm;
                ray[1] /= norm;
                ray[2] /= norm;
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        moment[r * 3 + c] += ray[r] * ray[c];
                    }
                }
                n++;
            }
            if (n < 6) {
                continue;
            }
            double lambda = smallestEigen(moment);
            sum += lambda / n;
            weight += 1;
        }
        if (weight < 4) {
            return 1e6;
        }
        double[] border = CalibrationManager.unprojectFisheye(
                p[2] + 0.48 * minSide, p[3], k, p[4], p[5], p[6], p[7]);
        if (border == null) {
            return 1e6;
        }
        double incDeg = Math.toDegrees(Math.atan2(
                Math.hypot(border[0], border[1]), Math.max(border[2], 1e-6)));
        if (incDeg < 70 || incDeg > 115) {
            return 1e6;
        }
        return sum / weight;
    }

    private static boolean paramsOk(double[] p, int w, int h, double minSide) {
        double fLo = VideoStreamingServer.fisheyeFocal(w, h, 215);
        double fHi = VideoStreamingServer.fisheyeFocal(w, h, 185);
        if (p[0] < fLo || p[0] > fHi || p[1] < fLo || p[1] > fHi) {
            return false;
        }
        if (Math.abs(p[0] - p[1]) > 0.05 * p[0]) {
            return false;
        }
        if (p[2] < 0.46 * w || p[2] > 0.54 * w || p[3] < 0.46 * h || p[3] > 0.54 * h) {
            return false;
        }
        if (Math.abs(p[4]) > 0.12 || Math.abs(p[5]) > 0.06
                || Math.abs(p[6]) > 0.03 || Math.abs(p[7]) > 0.02) {
            return false;
        }
        return !CalibrationManager.distortionFolds(p[4], p[5], p[6], p[7]);
    }

    /**
     * Forward 180° feed. Pixel x is azimuth, pixel y is elevation, both linear
     * in angle, so the side edges are exactly ±90° from the optical axis.
     */
    static Mat renderView(Mat src, Lens lens) {
        Mat mapX = new Mat(VIEW_H, VIEW_W, CvType.CV_32FC1);
        Mat mapY = new Mat(VIEW_H, VIEW_W, CvType.CV_32FC1);
        float[] rowX = new float[VIEW_W];
        float[] rowY = new float[VIEW_W];
        for (int y = 0; y < VIEW_H; y++) {
            for (int x = 0; x < VIEW_W; x++) {
                double[] ray = viewRay(x + 0.5, y + 0.5);
                float[] uv = projectRay(ray[0], ray[1], ray[2], lens);
                if (uv == null || uv[0] < 1 || uv[1] < 1
                        || uv[0] >= src.cols() - 1 || uv[1] >= src.rows() - 1) {
                    rowX[x] = -1f;
                    rowY[x] = -1f;
                } else {
                    rowX[x] = uv[0];
                    rowY[x] = uv[1];
                }
            }
            mapX.put(y, 0, rowX);
            mapY.put(y, 0, rowY);
        }
        Mat map1 = new Mat();
        Mat map2 = new Mat();
        Imgproc.convertMaps(mapX, mapY, map1, map2, CvType.CV_16SC2, false);
        mapX.release();
        mapY.release();
        Mat dst = new Mat();
        Imgproc.remap(src, dst, map1, map2, Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT, new Scalar(0, 0, 0));
        map1.release();
        map2.release();
        return dst;
    }

    private static void save(String role, int w, int h, Lens lens) {
        CalibrationManager.CameraModel model = new CalibrationManager.CameraModel(
                role, w, h, lens.after,
                lens.fx, lens.fy, lens.cx, lens.cy,
                lens.k1, lens.k2, lens.k3, lens.k4,
                true, false,
                nominalYaw(role), nominalPitch(role), 0,
                new double[] {0, 0, 0}, new double[] {0, 0, 0});
        if (!CalibrationManager.stableIntrinsics(model)) {
            System.out.println("  " + role + ": model outside the stable fisheye range, not saved");
            return;
        }
        CalibrationManager.save(model);
    }

    private static double nominalYaw(String role) {
        for (int i = 0; i < 4; i++) {
            if (VideoStreamingServer.CAM_ROLE[i].equals(role)) {
                return VideoStreamingServer.CAM_YAW_DEG[i];
            }
        }
        return 0;
    }

    private static double nominalPitch(String role) {
        for (int i = 0; i < 4; i++) {
            if (VideoStreamingServer.CAM_ROLE[i].equals(role)) {
                return VideoStreamingServer.CAM_PITCH_DEG[i];
            }
        }
        return 0;
    }

    /** ORB on the corrected views. Front yaw stays fixed. */
    private static void align(Mat[] views, Lens[] lenses, double[] yaw, double[] pitch, double[] roll) {
        int[][] pairs = { {1, 0, 0}, {1, 2, 2}, {2, 3, 3} };
        for (int[] pair : pairs) {
            int a = pair[0];
            int b = pair[1];
            int moving = pair[2];
            String label = VideoStreamingServer.CAM_ROLE[a] + "–" + VideoStreamingServer.CAM_ROLE[b];
            if (views[a] == null || views[b] == null) {
                System.out.println("  " + label + " skipped");
                continue;
            }
            List<double[]> raysA = new ArrayList<>();
            List<double[]> raysB = new ArrayList<>();
            matchPair(views[a], views[b], raysA, raysB);
            System.out.println("  " + label + "  ORB matches " + raysA.size());
            if (raysA.size() < 8) {
                continue;
            }
            double before = alignCost(raysA, raysB, yaw, pitch, roll, a, b);
            double baseY = yaw[moving];
            double baseP = pitch[moving];
            double best = before;
            double by = baseY;
            double bp = baseP;
            for (double dy = -8; dy <= 8.01; dy += 1) {
                for (double dp = -4; dp <= 4.01; dp += 1) {
                    yaw[moving] = baseY + dy;
                    pitch[moving] = baseP + dp;
                    double score = alignCost(raysA, raysB, yaw, pitch, roll, a, b);
                    if (score < best) {
                        best = score;
                        by = yaw[moving];
                        bp = pitch[moving];
                    }
                }
            }
            yaw[moving] = by;
            pitch[moving] = bp;
            System.out.printf(Locale.US, "  %s  ray error %.2f° → %.2f°  yaw %.1f pitch %.1f%n",
                    label, before, best, by, bp);
        }
    }

    private static void matchPair(Mat a, Mat b, List<double[]> raysA, List<double[]> raysB) {
        Mat grayA = new Mat();
        Mat grayB = new Mat();
        Imgproc.cvtColor(a, grayA, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(b, grayB, Imgproc.COLOR_BGR2GRAY);
        Mat maskA = sideMask(a.rows(), a.cols(), true);
        Mat maskB = sideMask(b.rows(), b.cols(), false);
        MatOfKeyPoint ka = new MatOfKeyPoint();
        MatOfKeyPoint kb = new MatOfKeyPoint();
        Mat da = new Mat();
        Mat db = new Mat();
        try {
            ORB orb = ORB.create(2000);
            orb.detectAndCompute(grayA, maskA, ka, da);
            orb.detectAndCompute(grayB, maskB, kb, db);
        } catch (Throwable t) {
            return;
        }
        if (da.empty() || db.empty()) {
            return;
        }
        List<MatOfDMatch> knn = new ArrayList<>();
        DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING).knnMatch(da, db, knn, 2);
        org.opencv.core.KeyPoint[] pa = ka.toArray();
        org.opencv.core.KeyPoint[] pb = kb.toArray();
        for (MatOfDMatch m : knn) {
            org.opencv.core.DMatch[] dm = m.toArray();
            if (dm.length < 2 || dm[0].distance > 0.75 * dm[1].distance) {
                continue;
            }
            double[] ra = viewRay(pa[dm[0].queryIdx].pt.x, pa[dm[0].queryIdx].pt.y);
            double[] rb = viewRay(pb[dm[0].trainIdx].pt.x, pb[dm[0].trainIdx].pt.y);
            if (ra != null && rb != null) {
                raysA.add(ra);
                raysB.add(rb);
            }
        }
        grayA.release();
        grayB.release();
        maskA.release();
        maskB.release();
        ka.release();
        kb.release();
        da.release();
        db.release();
    }

    private static Mat sideMask(int h, int w, boolean rightSide) {
        Mat mask = Mat.zeros(h, w, CvType.CV_8UC1);
        int x0 = rightSide ? (int) (w * 0.62) : 0;
        int x1 = rightSide ? w - 1 : (int) (w * 0.38);
        Imgproc.rectangle(mask, new org.opencv.core.Point(x0, 0),
                new org.opencv.core.Point(Math.max(x0 + 1, x1), h - 1),
                new Scalar(255), Imgproc.FILLED);
        return mask;
    }

    /** Azimuth −90°..+90° across the width. Elevation spans the 16:9 frame. */
    private static double[] viewRay(double u, double v) {
        double az = (u / VIEW_W - 0.5) * Math.toRadians(VIEW_HFOV_DEG);
        double el = (0.5 - v / VIEW_H) * Math.toRadians(VIEW_VFOV_DEG);
        double cosEl = Math.cos(el);
        return new double[] {
                cosEl * Math.sin(az),
                -Math.sin(el),
                cosEl * Math.cos(az)
        };
    }

    /** Kannala–Brandt projection of a camera ray, including the 90° rim. */
    private static float[] projectRay(double xc, double yc, double zc, Lens lens) {
        double norm = Math.hypot(xc, Math.hypot(yc, zc));
        if (norm < 1e-8) {
            return null;
        }
        xc /= norm;
        yc /= norm;
        zc /= norm;
        if (zc < 0) {
            return null;
        }
        double radial = Math.hypot(xc, yc);
        double theta = Math.atan2(radial, zc);
        double t2 = theta * theta;
        double t4 = t2 * t2;
        double t6 = t4 * t2;
        double t8 = t4 * t4;
        double thetaD = theta * (1.0 + lens.k1 * t2 + lens.k2 * t4 + lens.k3 * t6 + lens.k4 * t8);
        double deriv = 1.0 + 3.0 * lens.k1 * t2 + 5.0 * lens.k2 * t4
                + 7.0 * lens.k3 * t6 + 9.0 * lens.k4 * t8;
        if (deriv < 0.25 || thetaD < 0) {
            return null;
        }
        double scale = radial > 1e-8 ? thetaD / radial : 0;
        return new float[] {
                (float) (lens.fx * xc * scale + lens.cx),
                (float) (lens.fy * yc * scale + lens.cy)
        };
    }

    private static double alignCost(List<double[]> raysA, List<double[]> raysB,
                                    double[] yaw, double[] pitch, double[] roll, int a, int b) {
        double[] ra = VideoStreamingServer.cameraToVehicle(yaw[a], pitch[a], roll[a]);
        double[] rb = VideoStreamingServer.cameraToVehicle(yaw[b], pitch[b], roll[b]);
        List<Double> err = new ArrayList<>();
        for (int i = 0; i < raysA.size(); i++) {
            err.add(angle(mul(ra, raysA.get(i)), mul(rb, raysB.get(i))));
        }
        java.util.Collections.sort(err);
        int keep = Math.max(4, err.size() / 2);
        double sum = 0;
        for (int i = 0; i < keep; i++) {
            sum += Math.min(30.0, err.get(i));
        }
        return sum / keep;
    }

    private static double angle(double[] a, double[] b) {
        double na = Math.hypot(a[0], Math.hypot(a[1], a[2]));
        double nb = Math.hypot(b[0], Math.hypot(b[1], b[2]));
        double d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
        d = Math.max(-1, Math.min(1, d));
        return Math.toDegrees(Math.acos(d));
    }

    private static double[] mul(double[] r, double[] v) {
        return new double[] {
                r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
                r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
                r[6] * v[0] + r[7] * v[1] + r[8] * v[2]
        };
    }

    private static double smallestEigen(double[] m) {
        Mat mat = new Mat(3, 3, CvType.CV_64F);
        mat.put(0, 0, m);
        Mat eval = new Mat();
        Core.eigen(mat, eval);
        double lambda = eval.get(eval.rows() - 1, 0)[0];
        mat.release();
        eval.release();
        return Math.max(0, lambda);
    }

    private static double[] nelder(double[] x0, double[] step, int iterations, Scorer scorer) {
        int n = x0.length;
        double[][] s = new double[n + 1][n];
        double[] y = new double[n + 1];
        s[0] = x0.clone();
        y[0] = scorer.score(s[0]);
        for (int i = 0; i < n; i++) {
            s[i + 1] = x0.clone();
            s[i + 1][i] += step[i];
            y[i + 1] = scorer.score(s[i + 1]);
        }
        for (int iter = 0; iter < iterations; iter++) {
            int best = 0;
            int worst = 0;
            for (int i = 1; i <= n; i++) {
                if (y[i] < y[best]) {
                    best = i;
                }
                if (y[i] > y[worst]) {
                    worst = i;
                }
            }
            double[] cen = new double[n];
            for (int i = 0; i <= n; i++) {
                if (i == worst) {
                    continue;
                }
                for (int j = 0; j < n; j++) {
                    cen[j] += s[i][j];
                }
            }
            for (int j = 0; j < n; j++) {
                cen[j] /= n;
            }
            double[] refl = new double[n];
            for (int j = 0; j < n; j++) {
                refl[j] = cen[j] + (cen[j] - s[worst][j]);
            }
            double yr = scorer.score(refl);
            if (yr < y[best]) {
                double[] exp = new double[n];
                for (int j = 0; j < n; j++) {
                    exp[j] = cen[j] + 2 * (refl[j] - cen[j]);
                }
                double ye = scorer.score(exp);
                if (ye < yr) {
                    s[worst] = exp;
                    y[worst] = ye;
                } else {
                    s[worst] = refl;
                    y[worst] = yr;
                }
            } else if (yr < y[worst]) {
                s[worst] = refl;
                y[worst] = yr;
            } else {
                double[] con = new double[n];
                for (int j = 0; j < n; j++) {
                    con[j] = cen[j] + 0.5 * (s[worst][j] - cen[j]);
                }
                double yc = scorer.score(con);
                if (yc < y[worst]) {
                    s[worst] = con;
                    y[worst] = yc;
                } else {
                    for (int i = 0; i <= n; i++) {
                        if (i == best) {
                            continue;
                        }
                        for (int j = 0; j < n; j++) {
                            s[i][j] = 0.5 * (s[i][j] + s[best][j]);
                        }
                        y[i] = scorer.score(s[i]);
                    }
                }
            }
        }
        int best = 0;
        for (int i = 1; i <= n; i++) {
            if (y[i] < y[best]) {
                best = i;
            }
        }
        return s[best];
    }

    static int selfTest() {
        int w = 640;
        int h = 480;
        double f = VideoStreamingServer.fisheyeFocal(w, h, 200);
        double cx = w * 0.5;
        double cy = h * 0.5;
        double k1 = 0.08;
        double k2 = -0.02;
        double[] k = {f, f, cx, cy};
        List<double[][]> chains = new ArrayList<>();
        double[][][] lines = {
                { {0.2, -0.15, 1.4}, {0.55, 0.05, 0.15} },
                { {-0.3, 0.1, 1.6}, {0.2, 0.35, 0.05} },
                { {0.05, 0.25, 1.2}, {-0.4, 0.15, 0.2} },
                { {-0.15, -0.3, 1.5}, {0.45, 0.1, 0.1} }
        };
        for (double[][] line : lines) {
            List<double[]> pts = new ArrayList<>();
            for (int i = -10; i <= 10; i++) {
                double t = i / 10.0;
                double xc = line[0][0] + t * line[1][0];
                double yc = line[0][1] + t * line[1][1];
                double zc = line[0][2] + t * line[1][2];
                float[] uv = CalibrationManager.projectFisheye(xc, yc, zc, k, k1, k2, 0, 0);
                if (uv != null && uv[0] > 2 && uv[1] > 2 && uv[0] < w - 2 && uv[1] < h - 2) {
                    pts.add(new double[] {uv[0], uv[1]});
                }
            }
            if (pts.size() >= 6) {
                chains.add(pts.toArray(new double[0][]));
            }
        }
        double maxDeg = 0;
        for (double[][] chain : chains) {
            for (double[] px : chain) {
                double[] ray = CalibrationManager.unprojectFisheye(px[0], px[1], k, k1, k2, 0, 0);
                if (ray == null) {
                    continue;
                }
            }
        }
        double[] start = {f, f, cx, cy, 0, 0, 0, 0};
        double before = cost(chains, start, w, h, Math.min(w, h));
        double[] step = {f * 0.02, f * 0.02, 2, 2, 0.03, 0.015, 0.005, 0.004};
        double[] got = nelder(start, step, 40, q -> cost(chains, q, w, h, Math.min(w, h)));
        double after = cost(chains, got, w, h, Math.min(w, h));
        boolean ok = after < before * 0.85 && Math.abs(got[4] - k1) < 0.05;
        System.out.printf(Locale.US,
                "effective fisheye self-test  error %.5f → %.5f  k1=%.3f (truth %.3f)%n",
                before, after, got[4], k1);
        if (!ok) {
            System.err.println("effective fisheye self-test FAILED");
            return 1;
        }
        System.out.println("effective fisheye self-test OK");
        return 0;
    }

    private interface Scorer {
        double score(double[] p);
    }

    private static final class Lens {
        double fx, fy, cx, cy;
        double k1, k2, k3, k4;
        double fov;
        double before, after;
        int lines;
    }
}
