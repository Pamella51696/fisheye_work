import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/**
 * Final surround image from the four existing feeds.
 *
 *   representative frames
 *     → estimate K / D / FOV / center
 *     → fisheye correction
 *     → ground lines and ORB+RANSAC
 *     → camera pose on one vehicle ground plane
 *     → BEV
 *     → photometric correction
 *     → seam optimization
 *     → final output
 */
final class FinalOutput {

    private static final int[][] PAIRS = { {1, 0, 0}, {1, 2, 2}, {2, 3, 3} };

    private FinalOutput() {}

    static boolean isArgs(String[] args) {
        return has(args, "--final");
    }

    static boolean isSelfTestArgs(String[] args) {
        return has(args, "--final-self-test");
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

    static int run(Path folder) {
        System.out.println("FINAL OUTPUT");
        System.out.println("representative frames");
        Mat[] frames = VideoStreamingServer.representativeFrames(folder);
        if (frames == null) {
            return 2;
        }
        System.out.println("estimate effective fisheye parameters (K / D / FOV / center)");
        int estimated = LineFisheyeCalibrator.estimateIntrinsics(folder);
        if (estimated != 0) {
            System.out.println("  lens estimate did not finish; continuing with the current model");
        }
        for (Mat m : frames) {
            if (m != null) {
                m.release();
            }
        }
        frames = VideoStreamingServer.representativeFrames(folder);
        if (frames == null) {
            return 2;
        }
        reportLenses(frames);

        double[] yaw = new double[4];
        double[] pitch = new double[4];
        double[] roll = new double[4];
        loadPose(frames, yaw, pitch, roll);

        System.out.println("fisheye correction");
        GroundSurround.Lens[] lenses = lenses(frames, yaw, pitch, roll);
        for (int i = 0; i < 4; i++) {
            Mat corrected = GroundSurround.cameraBev(frames[i], lenses[i]);
            System.out.println("  " + VideoStreamingServer.CAM_ROLE[i]
                    + "  ground pixels " + groundPixels(corrected));
            corrected.release();
        }

        System.out.println("detect ground-plane features");
        System.out.println("  straight lines");
        System.out.println("  ORB + RANSAC");
        System.out.println("optimize camera pose");
        optimizePose(frames, yaw, pitch, roll);
        savePose(frames, yaw, pitch, roll);

        System.out.println("common vehicle ground plane");
        lenses = lenses(frames, yaw, pitch, roll);
        Mat[] bev = new Mat[4];
        int[] place = { 1, 0, 2, 3 };
        for (int n = 0; n < place.length; n++) {
            int i = place[n];
            if (i == 3) {
                System.out.println("rear");
            }
            bev[i] = GroundSurround.cameraBev(frames[i], lenses[i]);
            System.out.println("  " + VideoStreamingServer.CAM_ROLE[i]);
        }

        System.out.println("photometric correction");
        double[][] gains = photometric(bev);
        saveGains(gains);
        for (int i = 0; i < 4; i++) {
            System.out.printf(Locale.US, "  %s BGR gain %.3f %.3f %.3f%n",
                    VideoStreamingServer.CAM_ROLE[i], gains[i][0], gains[i][1], gains[i][2]);
        }

        System.out.println("seam optimization");
        optimizeSeams(frames, yaw, pitch, roll, gains);
        savePose(frames, yaw, pitch, roll);

        System.out.println("BEV");
        System.out.println("final output");
        lenses = lenses(frames, yaw, pitch, roll);
        GroundSurround surround = new GroundSurround();
        Mat out = surround.render(frames, lenses, gains);
        Path file = folder.resolve("final_output.jpg");
        boolean ok = Imgcodecs.imwrite(file.toAbsolutePath().toString(), out);
        out.release();
        for (Mat m : bev) {
            m.release();
        }
        for (Mat m : frames) {
            m.release();
        }
        if (!ok) {
            System.err.println("Could not write " + file);
            return 1;
        }
        System.out.println("Wrote " + file.toAbsolutePath());
        return 0;
    }

    private static int groundPixels(Mat bev) {
        int n = 0;
        for (int r = 0; r < bev.rows(); r += 4) {
            for (int c = 0; c < bev.cols(); c += 4) {
                double[] px = bev.get(r, c);
                if (px != null && px[0] + px[1] + px[2] > 8) {
                    n++;
                }
            }
        }
        return n * 16;
    }

    private static void reportLenses(Mat[] frames) {
        for (int i = 0; i < 4; i++) {
            GroundSurround.Lens lens = GroundSurround.Lens.fromCalibration(
                    i, frames[i].cols(), frames[i].rows());
            double fov = fovDeg(lens);
            System.out.printf(Locale.US,
                    "  %s  f=%.1f  center=(%.1f, %.1f)  D=[%.4f %.4f %.4f %.4f]  FOV=%.1f deg%n",
                    VideoStreamingServer.CAM_ROLE[i], lens.k[0], lens.k[2], lens.k[3],
                    lens.k1, lens.k2, lens.k3, lens.k4, fov);
        }
    }

    static double fovDeg(GroundSurround.Lens lens) {
        double radius = 0.5 * Math.min(lens.srcW, lens.srcH);
        double[] ray = CalibrationManager.unprojectFisheye(
                lens.k[2] + radius * 0.98, lens.k[3], lens.k, lens.k1, lens.k2, lens.k3, lens.k4);
        if (ray == null) {
            return Double.NaN;
        }
        double theta = Math.atan2(Math.hypot(ray[0], ray[1]), ray[2]);
        return Math.toDegrees(2.0 * theta);
    }

    private static void loadPose(Mat[] frames, double[] yaw, double[] pitch, double[] roll) {
        for (int i = 0; i < 4; i++) {
            CalibrationManager.CameraModel model =
                    CalibrationManager.load(VideoStreamingServer.CAM_ROLE[i]);
            double[] pose = CalibrationManager.mountDegreesForStitch(i, model, Double.NaN, Double.NaN);
            yaw[i] = pose[0];
            pitch[i] = pose[1];
            roll[i] = pose[2];
        }
    }

    private static void savePose(Mat[] frames, double[] yaw, double[] pitch, double[] roll) {
        for (int i = 0; i < 4; i++) {
            if (frames[i] == null) {
                continue;
            }
            CalibrationManager.CameraModel model =
                    CalibrationManager.load(VideoStreamingServer.CAM_ROLE[i]);
            if (model == null || !model.hasIntrinsics) {
                GroundSurround.Lens lens = GroundSurround.Lens.nominal(
                        i, frames[i].cols(), frames[i].rows());
                model = new CalibrationManager.CameraModel(
                        VideoStreamingServer.CAM_ROLE[i], frames[i].cols(), frames[i].rows(), 0,
                        lens.k[0], lens.k[1], lens.k[2], lens.k[3], 0, 0, 0, 0,
                        true, false,
                        VideoStreamingServer.CAM_YAW_DEG[i],
                        VideoStreamingServer.CAM_PITCH_DEG[i],
                        VideoStreamingServer.CAM_ROLL_DEG[i],
                        new double[] {0, 0, 0}, new double[] {0, 0, 0});
                CalibrationManager.save(model);
            }
            CalibrationManager.saveMountPose(
                    VideoStreamingServer.CAM_ROLE[i], yaw[i], pitch[i], roll[i]);
        }
    }

    private static GroundSurround.Lens[] lenses(Mat[] frames, double[] yaw, double[] pitch, double[] roll) {
        GroundSurround.Lens[] lenses = new GroundSurround.Lens[4];
        for (int i = 0; i < 4; i++) {
            lenses[i] = GroundSurround.Lens.posed(
                    i, frames[i].cols(), frames[i].rows(), yaw[i], pitch[i], roll[i]);
        }
        return lenses;
    }

    /** ORB matches and ground lines move left, right, and rear onto the front reference. */
    static void optimizePose(Mat[] frames, double[] yaw, double[] pitch, double[] roll) {
        for (int[] pair : PAIRS) {
            int anchor = pair[0];
            int other = pair[1];
            int moving = pair[2];
            String label = VideoStreamingServer.CAM_ROLE[anchor] + "–"
                    + VideoStreamingServer.CAM_ROLE[other];
            GroundSurround.Lens lensA = GroundSurround.Lens.posed(
                    anchor, frames[anchor].cols(), frames[anchor].rows(),
                    yaw[anchor], pitch[anchor], roll[anchor]);
            GroundSurround.Lens lensB = GroundSurround.Lens.posed(
                    other, frames[other].cols(), frames[other].rows(),
                    yaw[other], pitch[other], roll[other]);
            Mat bevA = GroundSurround.cameraBev(frames[anchor], lensA);
            Mat bevB = GroundSurround.cameraBev(frames[other], lensB);
            List<Seg> linesA = groundLines(bevA, lensA, anchor);
            List<Seg> linesB = groundLines(bevB, lensB, other);
            List<LinePair> linePairs = pairLines(linesA, linesB, yaw, pitch, roll);
            List<Tie> ties = orbTies(bevA, bevB, lensA, lensB, anchor, other);
            List<Tie> inliers = ransac(ties);
            bevA.release();
            bevB.release();
            System.out.println("  " + label + "  lines " + (linesA.size() + linesB.size())
                    + "  paired " + linePairs.size()
                    + "  ORB " + ties.size() + "  RANSAC inliers " + inliers.size());
            if (inliers.size() < 8 && linePairs.size() < 3) {
                System.out.println("  " + label + "  pose unchanged");
                continue;
            }
            double before = poseCost(inliers, linePairs, yaw, pitch, roll);
            searchMount(inliers, linePairs, moving, yaw, pitch, roll, 6, 4, 3, 2.0, 2.0, 1.0);
            searchMount(inliers, linePairs, moving, yaw, pitch, roll, 2, 2, 1, 0.5, 0.5, 0.5);
            clampMount(moving, yaw, pitch, roll);
            double after = poseCost(inliers, linePairs, yaw, pitch, roll);
            System.out.printf(Locale.US,
                    "  %s  ground error %.3f → %.3f m  yaw %.2f pitch %.2f roll %.2f%n",
                    label, before, after, yaw[moving], pitch[moving], roll[moving]);
        }
    }

    private static void searchMount(List<Tie> ties, int moving, double[] yaw, double[] pitch, double[] roll,
                                    double yawSpan, double pitchSpan, double rollSpan,
                                    double yawStep, double pitchStep, double rollStep) {
        searchMount(ties, Collections.<LinePair>emptyList(), moving, yaw, pitch, roll,
                yawSpan, pitchSpan, rollSpan, yawStep, pitchStep, rollStep);
    }

    private static void searchMount(List<Tie> ties, List<LinePair> lines, int moving,
                                    double[] yaw, double[] pitch, double[] roll,
                                    double yawSpan, double pitchSpan, double rollSpan,
                                    double yawStep, double pitchStep, double rollStep) {
        double baseY = yaw[moving];
        double baseP = pitch[moving];
        double baseR = roll[moving];
        double best = poseCost(ties, lines, yaw, pitch, roll);
        double by = baseY;
        double bp = baseP;
        double br = baseR;
        for (double dy = -yawSpan; dy <= yawSpan + 1e-6; dy += yawStep) {
            for (double dp = -pitchSpan; dp <= pitchSpan + 1e-6; dp += pitchStep) {
                for (double dr = -rollSpan; dr <= rollSpan + 1e-6; dr += rollStep) {
                    yaw[moving] = baseY + dy;
                    pitch[moving] = baseP + dp;
                    roll[moving] = baseR + dr;
                    double score = poseCost(ties, lines, yaw, pitch, roll);
                    if (score < best) {
                        best = score;
                        by = yaw[moving];
                        bp = pitch[moving];
                        br = roll[moving];
                    }
                }
            }
        }
        yaw[moving] = by;
        pitch[moving] = bp;
        roll[moving] = br;
    }

    private static double poseCost(List<Tie> ties, List<LinePair> lines,
                                   double[] yaw, double[] pitch, double[] roll) {
        double match = ties.size() >= 4 ? tieError(ties, yaw, pitch, roll) : 0;
        double line = linePenalty(lines, yaw, pitch, roll);
        if (ties.size() < 4 && lines.isEmpty()) {
            return 1e6;
        }
        return match + line;
    }

    private static void clampMount(int cam, double[] yaw, double[] pitch, double[] roll) {
        yaw[cam] = clamp(yaw[cam], VideoStreamingServer.CAM_YAW_DEG[cam] - 15,
                VideoStreamingServer.CAM_YAW_DEG[cam] + 15);
        pitch[cam] = clamp(pitch[cam], VideoStreamingServer.CAM_PITCH_DEG[cam] - 8,
                VideoStreamingServer.CAM_PITCH_DEG[cam] + 8);
        roll[cam] = clamp(roll[cam], VideoStreamingServer.CAM_ROLL_DEG[cam] - 6,
                VideoStreamingServer.CAM_ROLL_DEG[cam] + 6);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static List<Seg> groundLines(Mat bev, GroundSurround.Lens lens, int cam) {
        List<Seg> out = new ArrayList<>();
        Mat gray = new Mat();
        Imgproc.cvtColor(bev, gray, Imgproc.COLOR_BGR2GRAY);
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 50, 150);
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180.0, 40, 28, 12);
        for (int i = 0; i < lines.rows(); i++) {
            double[] s = lines.get(i, 0);
            if (s == null || s.length < 4) {
                continue;
            }
            double[] rayA = rayAt(lens, s[0], s[1]);
            double[] rayB = rayAt(lens, s[2], s[3]);
            if (rayA == null || rayB == null) {
                continue;
            }
            out.add(new Seg(cam, rayA, rayB, lens));
            if (out.size() >= 40) {
                break;
            }
        }
        gray.release();
        edges.release();
        lines.release();
        return out;
    }

    private static List<LinePair> pairLines(List<Seg> a, List<Seg> b,
                                            double[] yaw, double[] pitch, double[] roll) {
        List<LinePair> pairs = new ArrayList<>();
        for (Seg sa : a) {
            double[] midA = midpoint(sa, yaw, pitch, roll);
            if (midA == null) {
                continue;
            }
            Seg best = null;
            double bestD = 0.55;
            for (Seg sb : b) {
                double[] midB = midpoint(sb, yaw, pitch, roll);
                if (midB == null) {
                    continue;
                }
                double d = Math.hypot(midA[0] - midB[0], midA[1] - midB[1]);
                if (d < bestD && Math.abs(angleOf(sa, yaw, pitch, roll) - angleOf(sb, yaw, pitch, roll)) < 25) {
                    bestD = d;
                    best = sb;
                }
            }
            if (best != null) {
                pairs.add(new LinePair(sa, best));
            }
            if (pairs.size() >= 25) {
                break;
            }
        }
        return pairs;
    }

    private static double linePenalty(List<LinePair> lines, double[] yaw, double[] pitch, double[] roll) {
        if (lines == null || lines.isEmpty()) {
            return 0;
        }
        double sum = 0;
        int n = 0;
        for (LinePair p : lines) {
            double da = angleOf(p.a, yaw, pitch, roll);
            double db = angleOf(p.b, yaw, pitch, roll);
            if (Double.isNaN(da) || Double.isNaN(db)) {
                continue;
            }
            double diff = Math.abs(da - db);
            while (diff > 180) {
                diff -= 360;
            }
            diff = Math.abs(diff);
            if (diff > 90) {
                diff = 180 - diff;
            }
            sum += Math.min(30.0, diff);
            n++;
        }
        return n == 0 ? 0 : 0.01 * sum / n;
    }

    private static double[] midpoint(Seg seg, double[] yaw, double[] pitch, double[] roll) {
        GroundSurround.Lens lens = seg.lens.withMount(yaw[seg.cam], pitch[seg.cam], roll[seg.cam]);
        double[] a = GroundSurround.intersectGround(lens, seg.rayA);
        double[] b = GroundSurround.intersectGround(lens, seg.rayB);
        if (a == null || b == null) {
            return null;
        }
        return new double[] { 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]) };
    }

    private static double angleOf(Seg seg, double[] yaw, double[] pitch, double[] roll) {
        GroundSurround.Lens lens = seg.lens.withMount(yaw[seg.cam], pitch[seg.cam], roll[seg.cam]);
        double[] a = GroundSurround.intersectGround(lens, seg.rayA);
        double[] b = GroundSurround.intersectGround(lens, seg.rayB);
        if (a == null || b == null) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(b[1] - a[1], b[0] - a[0]));
    }

    private static List<Tie> orbTies(Mat bevA, Mat bevB, GroundSurround.Lens lensA, GroundSurround.Lens lensB,
                                     int camA, int camB) {
        List<Tie> ties = new ArrayList<>();
        Mat grayA = new Mat();
        Mat grayB = new Mat();
        Imgproc.cvtColor(bevA, grayA, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(bevB, grayB, Imgproc.COLOR_BGR2GRAY);
        Mat maskA = seamMask(bevA, camA, camB);
        Mat maskB = seamMask(bevB, camA, camB);
        MatOfKeyPoint keysA = new MatOfKeyPoint();
        MatOfKeyPoint keysB = new MatOfKeyPoint();
        Mat descA = new Mat();
        Mat descB = new Mat();
        try {
            ORB orb = ORB.create(2000);
            orb.detectAndCompute(grayA, maskA, keysA, descA);
            orb.detectAndCompute(grayB, maskB, keysB, descB);
        } catch (Throwable t) {
            release(grayA, grayB, maskA, maskB, keysA, keysB, descA, descB);
            return ties;
        }
        if (descA.empty() || descB.empty()) {
            release(grayA, grayB, maskA, maskB, keysA, keysB, descA, descB);
            return ties;
        }
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        List<MatOfDMatch> knn = new ArrayList<>();
        matcher.knnMatch(descA, descB, knn, 2);
        org.opencv.core.KeyPoint[] ka = keysA.toArray();
        org.opencv.core.KeyPoint[] kb = keysB.toArray();
        for (MatOfDMatch pair : knn) {
            org.opencv.core.DMatch[] dm = pair.toArray();
            if (dm.length < 2 || dm[0].distance > 0.75 * dm[1].distance) {
                continue;
            }
            double[] rayA = rayAt(lensA, ka[dm[0].queryIdx].pt.x, ka[dm[0].queryIdx].pt.y);
            double[] rayB = rayAt(lensB, kb[dm[0].trainIdx].pt.x, kb[dm[0].trainIdx].pt.y);
            if (rayA == null || rayB == null) {
                continue;
            }
            ties.add(new Tie(camA, camB, rayA, rayB, lensA, lensB));
        }
        release(grayA, grayB, maskA, maskB, keysA, keysB, descA, descB);
        return ties;
    }

    private static void release(Mat... mats) {
        for (Mat m : mats) {
            if (m != null) {
                m.release();
            }
        }
    }

    private static double[] rayAt(GroundSurround.Lens lens, double col, double row) {
        double[] ground = GroundSurround.groundOfPixel((int) Math.round(col), (int) Math.round(row));
        float[] uv = GroundSurround.projectGround(lens, ground[0], ground[1]);
        if (uv == null) {
            return null;
        }
        return CalibrationManager.unprojectFisheye(uv[0], uv[1], lens.k, lens.k1, lens.k2, lens.k3, lens.k4);
    }

    private static Mat seamMask(Mat bev, int camA, int camB) {
        Mat mask = Mat.zeros(bev.rows(), bev.cols(), CvType.CV_8UC1);
        byte[] row = new byte[bev.cols()];
        for (int r = 0; r < bev.rows(); r++) {
            for (int c = 0; c < bev.cols(); c++) {
                double[] g = GroundSurround.groundOfPixel(c, r);
                row[c] = (byte) (inSeam(camA, camB, g[0], g[1]) && bright(bev, c, r) ? 255 : 0);
            }
            mask.put(r, 0, row);
        }
        return mask;
    }

    private static boolean bright(Mat bgr, int col, int row) {
        double[] px = bgr.get(row, col);
        if (px == null) {
            return false;
        }
        return 0.1 * px[0] + 0.6 * px[1] + 0.3 * px[2] > 8;
    }

    static boolean inSeam(int camA, int camB, double x, double y) {
        double az = Math.atan2(y, x);
        double a = Math.toRadians(VideoStreamingServer.CAM_YAW_DEG[camA]);
        double b = Math.toRadians(VideoStreamingServer.CAM_YAW_DEG[camB]);
        double mid = a + wrap(b - a) / 2.0;
        double deg = Math.abs(Math.toDegrees(wrap(az - mid)));
        double radius = Math.hypot(x, y);
        return deg < 18 && radius > 2.6 && radius < 5.6
                && !GroundSurround.insideVehicle(x, y);
    }

    private static List<Tie> ransac(List<Tie> ties) {
        List<Tie> loose = new ArrayList<>();
        List<Tie> tight = new ArrayList<>();
        for (Tie t : ties) {
            GroundSurround.Lens lensA = t.lensA;
            GroundSurround.Lens lensB = t.lensB;
            double[] ga = GroundSurround.intersectGround(lensA, t.rayA);
            double[] gb = GroundSurround.intersectGround(lensB, t.rayB);
            if (ga == null || gb == null) {
                continue;
            }
            double d = Math.hypot(ga[0] - gb[0], ga[1] - gb[1]);
            if (d < 2.5) {
                loose.add(t);
            }
            if (d < 0.8) {
                tight.add(t);
            }
        }
        return tight.size() >= 8 ? tight : loose;
    }

    private static double tieError(List<Tie> ties, double[] yaw, double[] pitch, double[] roll) {
        List<Double> err = new ArrayList<>();
        for (Tie t : ties) {
            GroundSurround.Lens lensA = t.lensA.withMount(yaw[t.camA], pitch[t.camA], roll[t.camA]);
            GroundSurround.Lens lensB = t.lensB.withMount(yaw[t.camB], pitch[t.camB], roll[t.camB]);
            double[] ga = GroundSurround.intersectGround(lensA, t.rayA);
            double[] gb = GroundSurround.intersectGround(lensB, t.rayB);
            if (ga == null || gb == null) {
                err.add(4.0);
                continue;
            }
            err.add(Math.min(4.0, Math.hypot(ga[0] - gb[0], ga[1] - gb[1])));
        }
        if (err.size() < 4) {
            return 1e6;
        }
        Collections.sort(err);
        int keep = Math.max(4, err.size() / 2);
        double sum = 0;
        for (int i = 0; i < keep; i++) {
            sum += err.get(i);
        }
        return sum / keep;
    }

    /** Per-camera BGR gains so overlap seams share one brightness. Front stays 1. */
    static double[][] photometric(Mat[] bev) {
        double[][] gain = new double[4][3];
        for (int i = 0; i < 4; i++) {
            gain[i][0] = gain[i][1] = gain[i][2] = 1;
        }
        applyPairGain(bev, 1, 0, gain);
        applyPairGain(bev, 1, 2, gain);
        applyRearGain(bev, gain);
        return gain;
    }

    private static void applyPairGain(Mat[] bev, int anchor, int moving, double[][] gain) {
        double[] a = seamMean(bev[anchor], bev[moving], anchor, moving, true);
        double[] b = seamMean(bev[anchor], bev[moving], anchor, moving, false);
        for (int c = 0; c < 3; c++) {
            if (a[c] < 8 || b[c] < 8) {
                continue;
            }
            gain[moving][c] = clamp(a[c] / b[c], 0.7, 1.4);
        }
    }

    private static void applyRearGain(Mat[] bev, double[][] gain) {
        double[] fromLeft = new double[3];
        double[] fromRight = new double[3];
        double[] rearL = seamMean(bev[0], bev[3], 0, 3, false);
        double[] left = seamMean(bev[0], bev[3], 0, 3, true);
        double[] rearR = seamMean(bev[2], bev[3], 2, 3, false);
        double[] right = seamMean(bev[2], bev[3], 2, 3, true);
        int used = 0;
        for (int c = 0; c < 3; c++) {
            fromLeft[c] = (left[c] > 8 && rearL[c] > 8) ? (left[c] * gain[0][c]) / rearL[c] : 1;
            fromRight[c] = (right[c] > 8 && rearR[c] > 8) ? (right[c] * gain[2][c]) / rearR[c] : 1;
            double g = 0.5 * (fromLeft[c] + fromRight[c]);
            if (left[c] > 8 || right[c] > 8) {
                used++;
            }
            gain[3][c] = clamp(g, 0.7, 1.4);
        }
        if (used == 0) {
            gain[3][0] = gain[3][1] = gain[3][2] = 1;
        }
    }

    /** Mean BGR of one camera inside the seam. {@code first} selects bevA. */
    private static double[] seamMean(Mat a, Mat b, int camA, int camB, boolean first) {
        double[] sum = new double[3];
        int n = 0;
        for (int r = 0; r < a.rows(); r += 2) {
            for (int c = 0; c < a.cols(); c += 2) {
                double[] g = GroundSurround.groundOfPixel(c, r);
                if (!inSeam(camA, camB, g[0], g[1])) {
                    continue;
                }
                if (!bright(a, c, r) || !bright(b, c, r)) {
                    continue;
                }
                double[] px = (first ? a : b).get(r, c);
                sum[0] += px[0];
                sum[1] += px[1];
                sum[2] += px[2];
                n++;
            }
        }
        if (n == 0) {
            return new double[] {0, 0, 0};
        }
        return new double[] { sum[0] / n, sum[1] / n, sum[2] / n };
    }

    /** Nudge yaw so the brightness-matched seam has the smallest pixel difference. */
    static void optimizeSeams(Mat[] frames, double[] yaw, double[] pitch, double[] roll, double[][] gains) {
        for (int[] pair : PAIRS) {
            int anchor = pair[0];
            int other = pair[1];
            int moving = pair[2];
            String label = VideoStreamingServer.CAM_ROLE[anchor] + "–"
                    + VideoStreamingServer.CAM_ROLE[other];
            double base = yaw[moving];
            double best = base;
            double bestScore = seamScore(frames, yaw, pitch, roll, gains, anchor, other);
            for (double dy = -2; dy <= 2.0001; dy += 0.5) {
                yaw[moving] = base + dy;
                double score = seamScore(frames, yaw, pitch, roll, gains, anchor, other);
                if (score < bestScore) {
                    bestScore = score;
                    best = yaw[moving];
                }
            }
            yaw[moving] = best;
            clampMount(moving, yaw, pitch, roll);
            System.out.printf(Locale.US, "  %s seam %.2f → yaw %.2f  diff %.2f%n",
                    label, base, yaw[moving], bestScore);
        }
    }

    private static double seamScore(Mat[] frames, double[] yaw, double[] pitch, double[] roll,
                                    double[][] gains, int camA, int camB) {
        GroundSurround.Lens lensA = GroundSurround.Lens.posed(
                camA, frames[camA].cols(), frames[camA].rows(), yaw[camA], pitch[camA], roll[camA]);
        GroundSurround.Lens lensB = GroundSurround.Lens.posed(
                camB, frames[camB].cols(), frames[camB].rows(), yaw[camB], pitch[camB], roll[camB]);
        Mat a = GroundSurround.cameraBev(frames[camA], lensA);
        Mat b = GroundSurround.cameraBev(frames[camB], lensB);
        double sum = 0;
        int n = 0;
        for (int r = 0; r < a.rows(); r += 3) {
            for (int c = 0; c < a.cols(); c += 3) {
                double[] g = GroundSurround.groundOfPixel(c, r);
                if (!inSeam(camA, camB, g[0], g[1]) || !bright(a, c, r) || !bright(b, c, r)) {
                    continue;
                }
                double[] pa = a.get(r, c);
                double[] pb = b.get(r, c);
                for (int ch = 0; ch < 3; ch++) {
                    double va = pa[ch] * gains[camA][ch];
                    double vb = pb[ch] * gains[camB][ch];
                    sum += Math.abs(va - vb);
                }
                n++;
            }
        }
        a.release();
        b.release();
        return n < 20 ? 1e6 : sum / n;
    }

    static double[][] loadGains() {
        double[][] gain = new double[4][3];
        for (int i = 0; i < 4; i++) {
            gain[i][0] = gain[i][1] = gain[i][2] = 1;
        }
        Path file = Paths.get(CalibrationManager.CALIB_DIR).resolve("photometric.json");
        if (!Files.isRegularFile(file)) {
            return gain;
        }
        try {
            String json = Files.readString(file, StandardCharsets.UTF_8);
            int i = 0;
            for (int cam = 0; cam < 4; cam++) {
                for (int c = 0; c < 3; c++) {
                    int start = indexOfNumber(json, i);
                    if (start < 0) {
                        return gain;
                    }
                    int end = start;
                    while (end < json.length() && "0123456789.-eE".indexOf(json.charAt(end)) >= 0) {
                        end++;
                    }
                    gain[cam][c] = Double.parseDouble(json.substring(start, end));
                    i = end;
                }
            }
        } catch (Exception e) {
            System.err.println("Could not read photometric gains: " + e.getMessage());
        }
        return gain;
    }

    private static int indexOfNumber(String json, int from) {
        for (int i = from; i < json.length(); i++) {
            char ch = json.charAt(i);
            if (ch == '-' || (ch >= '0' && ch <= '9')) {
                return i;
            }
        }
        return -1;
    }

    static void saveGains(double[][] gains) {
        StringBuilder sb = new StringBuilder();
        sb.append("{\n  \"bgr\": [\n");
        for (int cam = 0; cam < 4; cam++) {
            sb.append(String.format(Locale.US, "    [%.6f, %.6f, %.6f]%s%n",
                    gains[cam][0], gains[cam][1], gains[cam][2], cam == 3 ? "" : ","));
        }
        sb.append("  ]\n}\n");
        Path file = Paths.get(CalibrationManager.CALIB_DIR).resolve("photometric.json");
        try {
            Files.createDirectories(file.getParent());
            Files.writeString(file, sb.toString(), StandardCharsets.UTF_8);
        } catch (Exception e) {
            System.err.println("Could not write " + file + ": " + e.getMessage());
        }
    }

    private static double wrap(double a) {
        while (a > Math.PI) {
            a -= 2 * Math.PI;
        }
        while (a < -Math.PI) {
            a += 2 * Math.PI;
        }
        return a;
    }

    /**
     * A ground point survives fisheye projection, and a yaw error on the right
     * camera is pulled back by ORB matches on the ground plane. A darker right
     * camera is brightened to the front camera.
     */
    static int selfTest() {
        boolean ok = true;
        ok &= roundTrip();
        ok &= poseAndPhotometric();
        if (!ok) {
            System.err.println("final-output self-test FAILED");
            return 1;
        }
        System.out.println("final-output self-test OK");
        return 0;
    }

    private static boolean roundTrip() {
        GroundSurround.Lens lens = GroundSurround.Lens.nominal(1, 640, 480);
        float[] uv = GroundSurround.projectGround(lens, 4.0, 0.3);
        if (uv == null) {
            System.err.println("final self-test: front camera missed ground point");
            return false;
        }
        double[] ray = CalibrationManager.unprojectFisheye(
                uv[0], uv[1], lens.k, lens.k1, lens.k2, lens.k3, lens.k4);
        double[] back = GroundSurround.intersectGround(lens, ray);
        if (back == null || Math.hypot(back[0] - 4.0, back[1] - 0.3) > 0.03) {
            System.err.println("final self-test: ground round-trip failed");
            return false;
        }
        return true;
    }

    private static boolean poseAndPhotometric() {
        int w = 480;
        int h = 360;
        GroundSurround.Lens front = GroundSurround.Lens.nominal(1, w, h);
        GroundSurround.Lens right = GroundSurround.Lens.nominal(2, w, h);
        Mat frontFrame = synthesize(front);
        Mat rightFrame = synthesize(right);
        double[] yaw = VideoStreamingServer.CAM_YAW_DEG.clone();
        double[] pitch = VideoStreamingServer.CAM_PITCH_DEG.clone();
        double[] roll = VideoStreamingServer.CAM_ROLL_DEG.clone();
        yaw[2] += 4.0;
        Mat[] onePair = new Mat[] { null, frontFrame, rightFrame, null };
        List<Tie> ties = measure(onePair, yaw, pitch, roll, 1, 2);
        if (ties.size() < 8) {
            System.err.println("final self-test: ORB found " + ties.size() + " ground matches");
            frontFrame.release();
            rightFrame.release();
            return false;
        }
        double before = tieError(ties, yaw, pitch, roll);
        searchMount(ties, 2, yaw, pitch, roll, 6, 0, 0, 1.0, 1, 1);
        searchMount(ties, 2, yaw, pitch, roll, 1.5, 0, 0, 0.5, 1, 1);
        double after = tieError(ties, yaw, pitch, roll);
        System.out.printf(Locale.US, "  pose recovery yaw %.2f  error %.3f → %.3f m%n",
                yaw[2], before, after);
        boolean poseOk = Math.abs(yaw[2] - VideoStreamingServer.CAM_YAW_DEG[2]) < 1.6 && after < before;
        Mat lineBev = GroundSurround.cameraBev(frontFrame, front);
        int lineCount = groundLines(lineBev, front, 1).size();
        lineBev.release();
        if (lineCount < 1) {
            System.err.println("final self-test: no ground lines on the corrected view");
            poseOk = false;
        }

        yaw[2] = VideoStreamingServer.CAM_YAW_DEG[2];
        GroundSurround.Lens[] lenses = new GroundSurround.Lens[4];
        lenses[1] = front;
        lenses[2] = right;
        Mat[] bev = new Mat[4];
        Mat darkRight = scale(rightFrame, 0.55);
        bev[1] = GroundSurround.cameraBev(frontFrame, lenses[1]);
        bev[2] = GroundSurround.cameraBev(darkRight, lenses[2]);
        darkRight.release();
        double[][] gains = new double[4][3];
        for (int i = 0; i < 4; i++) {
            gains[i] = new double[] {1, 1, 1};
        }
        applyPairGain(bev, 1, 2, gains);
        boolean photoOk = gains[2][1] > 1.15;
        System.out.printf(Locale.US, "  photometric right gain %.3f%n", gains[2][1]);
        if (!photoOk) {
            System.err.println("final self-test: photometric gain did not lift the dark camera");
        }
        for (Mat m : bev) {
            if (m != null) {
                m.release();
            }
        }
        frontFrame.release();
        rightFrame.release();
        return poseOk && photoOk;
    }

    private static List<Tie> measure(Mat[] frames, double[] yaw, double[] pitch, double[] roll,
                                     int camA, int camB) {
        GroundSurround.Lens lensA = GroundSurround.Lens.nominal(camA, frames[camA].cols(), frames[camA].rows())
                .withMount(yaw[camA], pitch[camA], roll[camA]);
        GroundSurround.Lens lensB = GroundSurround.Lens.nominal(camB, frames[camB].cols(), frames[camB].rows())
                .withMount(yaw[camB], pitch[camB], roll[camB]);
        Mat bevA = GroundSurround.cameraBev(frames[camA], lensA);
        Mat bevB = GroundSurround.cameraBev(frames[camB], lensB);
        List<Tie> ties = orbTies(bevA, bevB, lensA, lensB, camA, camB);
        bevA.release();
        bevB.release();
        List<Tie> inliers = ransac(ties);
        return inliers.size() >= 8 ? inliers : ties;
    }

    private static Mat scale(Mat src, double gain) {
        Mat dst = new Mat();
        src.convertTo(dst, -1, gain, 0);
        return dst;
    }

    private static Mat synthesize(GroundSurround.Lens lens) {
        int w = lens.srcW;
        int h = lens.srcH;
        byte[] buf = new byte[w * h * 3];
        for (int v = 0; v < h; v++) {
            for (int u = 0; u < w; u++) {
                double[] ray = CalibrationManager.unprojectFisheye(
                        u, v, lens.k, lens.k1, lens.k2, lens.k3, lens.k4);
                if (ray == null) {
                    continue;
                }
                double[] g = GroundSurround.intersectGround(lens, ray);
                if (g == null) {
                    continue;
                }
                int i = (v * w + u) * 3;
                byte[] bgr = texture(g[0], g[1]);
                buf[i] = bgr[0];
                buf[i + 1] = bgr[1];
                buf[i + 2] = bgr[2];
            }
        }
        Mat img = new Mat(h, w, CvType.CV_8UC3);
        img.put(0, 0, buf);
        return img;
    }

    private static byte[] texture(double x, double y) {
        double[][] marks = {
                {3.6, 0.6}, {3.2, 1.5}, {2.9, -0.7}, {4.1, -1.2},
                {1.2, 3.4}, {0.4, 4.0}, {-0.6, 3.1}, {2.2, 2.6}
        };
        for (int i = 0; i < marks.length; i++) {
            if (Math.hypot(x - marks[i][0], y - marks[i][1]) < 0.22) {
                int v = 40 + i * 24;
                return new byte[] { (byte) v, (byte) (255 - v), (byte) (80 + i * 10) };
            }
        }
        int sx = (int) Math.floor((x + 8) / 0.45);
        int sy = (int) Math.floor((y + 8) / 0.45);
        int v = ((sx + sy) & 1) == 0 ? 210 : 25;
        return new byte[] { (byte) v, (byte) v, (byte) (v - 10) };
    }

    private static final class Tie {
        final int camA;
        final int camB;
        final double[] rayA;
        final double[] rayB;
        final GroundSurround.Lens lensA;
        final GroundSurround.Lens lensB;

        Tie(int camA, int camB, double[] rayA, double[] rayB,
            GroundSurround.Lens lensA, GroundSurround.Lens lensB) {
            this.camA = camA;
            this.camB = camB;
            this.rayA = rayA;
            this.rayB = rayB;
            this.lensA = lensA;
            this.lensB = lensB;
        }
    }

    private static final class Seg {
        final int cam;
        final double[] rayA;
        final double[] rayB;
        final GroundSurround.Lens lens;

        Seg(int cam, double[] rayA, double[] rayB, GroundSurround.Lens lens) {
            this.cam = cam;
            this.rayA = rayA;
            this.rayB = rayB;
            this.lens = lens;
        }
    }

    private static final class LinePair {
        final Seg a;
        final Seg b;

        LinePair(Seg a, Seg b) {
            this.a = a;
            this.b = b;
        }
    }
}
