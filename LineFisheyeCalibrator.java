import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.LineSegmentDetector;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

/**
 * Main calibration path. Uses the existing surround clips, not a checkerboard.
 *
 *   existing video
 *     → fisheye initialization
 *     → straight-line calibration
 *     → initial K,D
 *     → undistortion / projection
 *     → ORB feature matching
 *     → RANSAC
 *     → extrinsic refinement
 *     → ground-line optimization
 *     → final calibration
 *
 * Checkerboard calibration stays available as {@code --calibrate-chessboard}.
 */
public final class LineFisheyeCalibrator {

    private LineFisheyeCalibrator() {}

    static boolean isSelfTestArgs(String[] args) {
        if (args == null) {
            return false;
        }
        for (String a : args) {
            if ("--line-self-test".equals(a)) {
                return true;
            }
        }
        return false;
    }

    static int run(java.nio.file.Path folder) {
        System.out.println("LINE CALIBRATION (main)");
        System.out.println("Existing video → straight lines → K,D → ORB → RANSAC → ground lines");
        System.out.println("Working folder: " + folder);
        System.out.println();

        java.nio.file.Path[] clips = VideoStreamingServer.discoverClips(folder);
        if (clips == null) {
            return 2;
        }

        int saved = 0;
        boolean[] wrote = new boolean[4];
        @SuppressWarnings("unchecked")
        List<Chain>[] chains = new List[4];
        Mat[] preview = new Mat[4];
        for (int i = 0; i < 4; i++) {
            String role = VideoStreamingServer.CAM_ROLE[i];
            System.out.println("── " + role.toUpperCase(Locale.ROOT) + " ──");
            System.out.println("existing video: " + clips[i].getFileName());
            List<Mat> frames = sampleFrames(clips[i], 4);
            if (frames.isEmpty()) {
                System.err.println("Could not read frames from " + clips[i].getFileName());
                continue;
            }
            preview[i] = frames.get(0);
            int w = frames.get(0).cols();
            int h = frames.get(0).rows();
            Fit fit = fitCamera(frames, w, h);
            chains[i] = fit.chains;
            if (fit.accepted) {
                CalibrationManager.CameraModel model = new CalibrationManager.CameraModel(
                        role, w, h, fit.rmsDeg, fit.f, fit.f, fit.cx, fit.cy,
                        fit.k1, fit.k2, fit.k3, fit.k4,
                        true, false,
                        VideoStreamingServer.CAM_YAW_DEG[i],
                        VideoStreamingServer.CAM_PITCH_DEG[i],
                        VideoStreamingServer.CAM_ROLL_DEG[i],
                        new double[] {0, 0, 0}, new double[] {0, 0, 0});
                if (CalibrationManager.save(model)) {
                    saved++;
                    wrote[i] = true;
                    System.out.printf(Locale.US,
                            "initial K,D  f=%.2f cx=%.1f cy=%.1f  D=[%.4f %.4f %.4f %.4f]  line-rms=%.3f deg%n",
                            fit.f, fit.cx, fit.cy, fit.k1, fit.k2, fit.k3, fit.k4, fit.rmsDeg);
                }
            } else {
                System.err.println("Straight-line fit did not beat the FOV initialization for " + role + ".");
                System.err.println("Extrinsics will use the equidistant model. "
                        + "A checkerboard pass is available: --calibrate-chessboard");
            }
            for (int f = 1; f < frames.size(); f++) {
                frames.get(f).release();
            }
            System.out.println();
        }

        System.out.println("undistortion/projection");
        for (int i = 0; i < 4; i++) {
            if (preview[i] == null) {
                continue;
            }
            Mat panel = VideoStreamingServer.projectPanel(preview[i], i);
            int valid = coverage(panel);
            System.out.println("  " + VideoStreamingServer.CAM_ROLE[i]
                    + " projected coverage " + valid + " px");
            panel.release();
        }
        System.out.println();

        System.out.println("ORB feature matching");
        List<RayMatch> matches = matchAdjacent(preview);
        System.out.println("  matches: " + matches.size());
        if (matches.size() >= 12) {
            double[] yaw = new double[4];
            double[] pitch = new double[4];
            double[] roll = new double[4];
            for (int i = 0; i < 4; i++) {
                CalibrationManager.CameraModel m = CalibrationManager.load(VideoStreamingServer.CAM_ROLE[i]);
                double[] pose = CalibrationManager.mountDegreesForStitch(i, m, Double.NaN, Double.NaN);
                yaw[i] = pose[0];
                pitch[i] = pose[1];
                roll[i] = pose[2];
            }
            System.out.println("RANSAC");
            ransacExtrinsics(matches, yaw, pitch, roll);
            System.out.println("extrinsic refinement");
            refineExtrinsics(matches, yaw, pitch, roll);
            System.out.println("ground-line optimization");
            optimizeGroundLines(chains, preview, yaw, pitch, roll);
            System.out.println("final calibration");
            for (int i = 0; i < 4; i++) {
                yaw[i] = clamp(yaw[i], VideoStreamingServer.CAM_YAW_DEG[i] - 10,
                        VideoStreamingServer.CAM_YAW_DEG[i] + 10);
                pitch[i] = clamp(pitch[i], VideoStreamingServer.CAM_PITCH_DEG[i] - 8,
                        VideoStreamingServer.CAM_PITCH_DEG[i] + 8);
                roll[i] = clamp(roll[i], VideoStreamingServer.CAM_ROLL_DEG[i] - 6,
                        VideoStreamingServer.CAM_ROLL_DEG[i] + 6);
                if (!wrote[i]) {
                    continue;
                }
                CalibrationManager.saveMountPose(
                        VideoStreamingServer.CAM_ROLE[i], yaw[i], pitch[i], roll[i]);
                System.out.printf(Locale.US, "  %s yaw=%.2f pitch=%.2f roll=%.2f%n",
                        VideoStreamingServer.CAM_ROLE[i], yaw[i], pitch[i], roll[i]);
            }
        } else {
            System.err.println("Not enough overlap matches for extrinsics.");
            System.err.println("Intrinsics were still saved when the line fit succeeded.");
        }

        for (Mat m : preview) {
            if (m != null) {
                m.release();
            }
        }
        System.out.println();
        System.out.println("Line calibration finished. cameras with a line-based K,D: " + saved);
        return saved > 0 || matches.size() >= 12 ? 0 : 2;
    }

    private static final class Chain {
        final List<double[]> pts = new ArrayList<>();
        double length;
        int imageHeight;
    }

    private static final class Fit {
        double f;
        double cx;
        double cy;
        double k1, k2, k3, k4;
        double rmsDeg;
        boolean accepted;
        List<Chain> chains = new ArrayList<>();
    }

    private static final class RayMatch {
        final int a;
        final int b;
        final double[] rayA;
        final double[] rayB;

        RayMatch(int a, int b, double[] rayA, double[] rayB) {
            this.a = a;
            this.b = b;
            this.rayA = rayA;
            this.rayB = rayB;
        }
    }

    private static Fit fitCamera(List<Mat> frames, int w, int h) {
        Fit fit = new Fit();
        double minSide = Math.min(w, h);
        fit.f = VideoStreamingServer.fisheyeFocal(w, h, VideoStreamingServer.INPUT_FISHEYE_FOV_DEG);
        fit.cx = w * VideoStreamingServer.FISHEYE_CX;
        fit.cy = h * VideoStreamingServer.FISHEYE_CY;
        System.out.println("fisheye model initialization");
        System.out.printf(Locale.US, "  f=%.2f cx=%.1f cy=%.1f D=[0 0 0 0]%n", fit.f, fit.cx, fit.cy);

        System.out.println("detect straight lines");
        for (Mat frame : frames) {
            fit.chains.addAll(detectChains(frame));
        }
        fit.chains.sort(Comparator.comparingDouble((Chain c) -> c.length).reversed());
        if (fit.chains.size() > 60) {
            fit.chains = new ArrayList<>(fit.chains.subList(0, 60));
        }
        System.out.println("  line chains: " + fit.chains.size());
        if (fit.chains.size() < 4) {
            System.err.println("Need long straight edges (lanes, curbs, buildings). Found "
                    + fit.chains.size() + ".");
            return fit;
        }

        System.out.println("straight-line calibration");
        System.out.println("fit fisheye model");
        final double fRef = fit.f;
        double initCost = lineCost(fit.chains, fit.f, fit.cx, fit.cy, 0, 0, 0, 0, minSide);
        double[] best = {fit.f, fit.cx, fit.cy, 0, 0, 0, 0};
        double bestCost = initCost;
        double[] k1Seeds = {0, 0.05, -0.05};
        for (double k1 : k1Seeds) {
            double[] start = {fit.f, fit.cx, fit.cy, k1, 0, 0, 0};
            double[] step = {fit.f * 0.04, w * 0.01, h * 0.01, 0.02, 0.015, 0, 0};
            double[] got = nelderMead(start, step, 28,
                    p -> evalParams(p, fit.chains, w, h, minSide, fRef));
            double c = evalParams(got, fit.chains, w, h, minSide, fRef);
            if (c < bestCost) {
                bestCost = c;
                best = got;
            }
        }
        double[] step = {best[0] * 0.025, w * 0.008, h * 0.008, 0.012, 0.01, 0, 0};
        best = nelderMead(best, step, 40, p -> evalParams(p, fit.chains, w, h, minSide, fRef));
        bestCost = evalParams(best, fit.chains, w, h, minSide, fRef);
        fit.f = best[0];
        fit.cx = best[1];
        fit.cy = best[2];
        fit.k1 = best[3];
        fit.k2 = best[4];
        fit.k3 = 0;
        fit.k4 = 0;
        fit.rmsDeg = Math.toDegrees(Math.sqrt(Math.max(0, bestCost)));
        fit.accepted = bestCost < initCost * 0.97 && Double.isFinite(bestCost)
                && !CalibrationManager.distortionFolds(fit.k1, fit.k2, 0, 0);
        System.out.printf(Locale.US, "  plane error init %.5f → fit %.5f%n", initCost, bestCost);
        return fit;
    }

    private static double evalParams(double[] p, List<Chain> chains, int w, int h,
                                     double minSide, double fRef) {
        double f = p[0];
        double cx = p[1];
        double cy = p[2];
        if (f < fRef * 0.88 || f > fRef * 1.12) {
            return 1e6;
        }
        if (cx < 0.45 * w || cx > 0.55 * w || cy < 0.45 * h || cy > 0.55 * h) {
            return 1e6;
        }
        if (Math.abs(p[3]) > 0.12 || Math.abs(p[4]) > 0.05
                || Math.abs(p[5]) > 1e-6 || Math.abs(p[6]) > 1e-6) {
            return 1e6;
        }
        if (CalibrationManager.distortionFolds(p[3], p[4], 0, 0)) {
            return 1e6;
        }
        return lineCost(chains, f, cx, cy, p[3], p[4], 0, 0, minSide);
    }

    /** Mean squared distance of unprojected rays from a plane through the origin. */
    private static double lineCost(List<Chain> chains, double f, double cx, double cy,
                                   double k1, double k2, double k3, double k4, double minSide) {
        double[] k = {f, f, cx, cy};
        double sum = 0;
        double weight = 0;
        for (Chain chain : chains) {
            double[] moment = new double[9];
            int n = 0;
            for (double[] px : chain.pts) {
                double[] ray = CalibrationManager.unprojectFisheye(px[0], px[1], k, k1, k2, k3, k4);
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
            double w = chain.length;
            sum += w * (lambda / n);
            weight += w;
        }
        if (weight <= 0) {
            return 1e6;
        }
        // A too-large focal length flattens every ray toward the optical axis and
        // makes the plane error look good. Hold the border incidence near a fisheye FOV.
        double[] border = CalibrationManager.unprojectFisheye(
                cx + 0.48 * minSide, cy, k, k1, k2, k3, k4);
        double penalty = 0;
        if (border != null) {
            double inc = Math.atan2(Math.hypot(border[0], border[1]), Math.max(border[2], 1e-6));
            double target = Math.toRadians(84);
            double d = inc - target;
            penalty = 0.15 * d * d;
        }
        return sum / weight + penalty;
    }

    private static double angleDist(double a, double b) {
        double d = Math.abs(a - b);
        while (d > Math.PI) {
            d -= Math.PI;
        }
        if (d > Math.PI / 2) {
            d = Math.PI - d;
        }
        return d;
    }

    private static double smallestEigen(double[] m) {
        double trace = m[0] + m[4] + m[8];
        double[] b = {
                trace - m[0], -m[1], -m[2],
                -m[3], trace - m[4], -m[5],
                -m[6], -m[7], trace - m[8]
        };
        double[] v = {0.7, 0.4, 0.2};
        for (int iter = 0; iter < 16; iter++) {
            double x = b[0] * v[0] + b[1] * v[1] + b[2] * v[2];
            double y = b[3] * v[0] + b[4] * v[1] + b[5] * v[2];
            double z = b[6] * v[0] + b[7] * v[1] + b[8] * v[2];
            double n = Math.hypot(x, Math.hypot(y, z));
            if (n < 1e-12) {
                break;
            }
            v[0] = x / n;
            v[1] = y / n;
            v[2] = z / n;
        }
        return v[0] * (m[0] * v[0] + m[1] * v[1] + m[2] * v[2])
                + v[1] * (m[3] * v[0] + m[4] * v[1] + m[5] * v[2])
                + v[2] * (m[6] * v[0] + m[7] * v[1] + m[8] * v[2]);
    }

    private static List<Chain> detectChains(Mat bgr) {
        Mat gray = new Mat();
        if (bgr.channels() == 1) {
            bgr.copyTo(gray);
        } else {
            Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
        }
        List<double[]> segs = new ArrayList<>();
        Mat lines = new Mat();
        try {
            LineSegmentDetector lsd = Imgproc.createLineSegmentDetector();
            lsd.detect(gray, lines);
        } catch (Throwable t) {
            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 60, 160);
            Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180.0, 40, 40, 18);
            edges.release();
        }
        for (int i = 0; i < lines.rows(); i++) {
            double[] v = lines.get(i, 0);
            if (v == null || v.length < 4) {
                continue;
            }
            double len = Math.hypot(v[2] - v[0], v[3] - v[1]);
            if (len >= 28) {
                segs.add(new double[] {v[0], v[1], v[2], v[3]});
            }
        }
        lines.release();
        gray.release();
        return chainSegments(segs, bgr.cols(), bgr.rows());
    }

    private static List<Chain> chainSegments(List<double[]> segs, int w, int h) {
        int n = segs.size();
        boolean[] used = new boolean[n];
        double diag = Math.hypot(w, h);
        List<Chain> chains = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (used[i]) {
                continue;
            }
            used[i] = true;
            Chain chain = new Chain();
            chain.imageHeight = h;
            double[] s = segs.get(i);
            sampleSegment(chain.pts, s[0], s[1], s[2], s[3], 12);
            chain.length = Math.hypot(s[2] - s[0], s[3] - s[1]);
            double tx = s[2];
            double ty = s[3];
            double hx = s[0];
            double hy = s[1];
            double tailAng = Math.atan2(s[3] - s[1], s[2] - s[0]);
            double headAng = tailAng;
            boolean grew = true;
            while (grew) {
                grew = false;
                int best = -1;
                double bestD = 40;
                boolean toTail = true;
                boolean flip = false;
                for (int j = 0; j < n; j++) {
                    if (used[j]) {
                        continue;
                    }
                    double[] q = segs.get(j);
                    double qAng = Math.atan2(q[3] - q[1], q[2] - q[0]);
                    boolean tailOk = angleDist(qAng, tailAng) <= Math.toRadians(18);
                    boolean headOk = angleDist(qAng, headAng) <= Math.toRadians(18);
                    if (!tailOk && !headOk) {
                        continue;
                    }
                    double dTailStart = tailOk ? Math.hypot(q[0] - tx, q[1] - ty) : 1e9;
                    double dTailEnd = tailOk ? Math.hypot(q[2] - tx, q[3] - ty) : 1e9;
                    double dHeadStart = headOk ? Math.hypot(q[0] - hx, q[1] - hy) : 1e9;
                    double dHeadEnd = headOk ? Math.hypot(q[2] - hx, q[3] - hy) : 1e9;
                    if (dTailStart < bestD) {
                        bestD = dTailStart;
                        best = j;
                        toTail = true;
                        flip = false;
                    }
                    if (dTailEnd < bestD) {
                        bestD = dTailEnd;
                        best = j;
                        toTail = true;
                        flip = true;
                    }
                    if (dHeadStart < bestD) {
                        bestD = dHeadStart;
                        best = j;
                        toTail = false;
                        flip = true;
                    }
                    if (dHeadEnd < bestD) {
                        bestD = dHeadEnd;
                        best = j;
                        toTail = false;
                        flip = false;
                    }
                }
                if (best < 0) {
                    break;
                }
                used[best] = true;
                double[] q = segs.get(best);
                double x1 = flip ? q[2] : q[0];
                double y1 = flip ? q[3] : q[1];
                double x2 = flip ? q[0] : q[2];
                double y2 = flip ? q[1] : q[3];
                if (!toTail) {
                    double sx = x1;
                    double sy = y1;
                    x1 = x2;
                    y1 = y2;
                    x2 = sx;
                    y2 = sy;
                    hx = x2;
                    hy = y2;
                    headAng = Math.atan2(y1 - y2, x1 - x2);
                } else {
                    tx = x2;
                    ty = y2;
                    tailAng = Math.atan2(y2 - y1, x2 - x1);
                }
                sampleSegment(chain.pts, x1, y1, x2, y2, 12);
                chain.length += Math.hypot(q[2] - q[0], q[3] - q[1]);
                grew = true;
            }
            if (chain.length > 0.12 * diag && chain.pts.size() >= 8) {
                chains.add(chain);
            }
        }
        return chains;
    }

    private static void sampleSegment(List<double[]> pts, double x1, double y1,
                                      double x2, double y2, double step) {
        double len = Math.hypot(x2 - x1, y2 - y1);
        int n = Math.max(1, (int) Math.ceil(len / step));
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n;
            pts.add(new double[] {x1 + (x2 - x1) * t, y1 + (y2 - y1) * t});
        }
    }

    private static List<Mat> sampleFrames(java.nio.file.Path clip, int want) {
        List<Mat> frames = new ArrayList<>();
        java.nio.file.Path decodable = VideoStreamingServer.ensureDecodable(clip);
        VideoCapture cap = VideoStreamingServer.openVideo(decodable);
        if (cap == null || !cap.isOpened()) {
            return frames;
        }
        double count = cap.get(Videoio.CAP_PROP_FRAME_COUNT);
        if (count >= want * 2) {
            for (int i = 0; i < want; i++) {
                cap.set(Videoio.CAP_PROP_POS_FRAMES, (i + 0.5) / want * (count - 1));
                Mat frame = new Mat();
                if (VideoStreamingServer.readBgr(cap, frame)) {
                    frames.add(frame);
                } else {
                    frame.release();
                }
            }
        }
        if (frames.isEmpty()) {
            cap.set(Videoio.CAP_PROP_POS_FRAMES, 0);
            Mat frame = new Mat();
            int seen = 0;
            while (frames.size() < want && VideoStreamingServer.readBgr(cap, frame)) {
                if (seen % 8 == 0) {
                    frames.add(frame);
                    frame = new Mat();
                }
                seen++;
            }
            frame.release();
        }
        cap.release();
        return frames;
    }

    private static int coverage(Mat bgr) {
        if (bgr == null || bgr.empty()) {
            return 0;
        }
        Mat gray = new Mat();
        Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
        Mat mask = new Mat();
        Imgproc.threshold(gray, mask, 3, 255, Imgproc.THRESH_BINARY);
        int n = Core.countNonZero(mask);
        gray.release();
        mask.release();
        return n;
    }

    private static List<RayMatch> matchAdjacent(Mat[] frames) {
        List<RayMatch> out = new ArrayList<>();
        ORB orb = ORB.create(1800);
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        MatOfKeyPoint[] keys = new MatOfKeyPoint[4];
        Mat[] desc = new Mat[4];
        Mat[] gray = new Mat[4];
        for (int i = 0; i < 4; i++) {
            keys[i] = new MatOfKeyPoint();
            desc[i] = new Mat();
            gray[i] = new Mat();
            if (frames[i] == null || frames[i].empty()) {
                continue;
            }
            Imgproc.cvtColor(frames[i], gray[i], Imgproc.COLOR_BGR2GRAY);
            orb.detectAndCompute(gray[i], new Mat(), keys[i], desc[i]);
        }
        for (int i = 0; i < 3; i++) {
            if (desc[i].empty() || desc[i + 1].empty()) {
                continue;
            }
            List<MatOfDMatch> knn = new ArrayList<>();
            matcher.knnMatch(desc[i], desc[i + 1], knn, 2);
            org.opencv.core.KeyPoint[] ka = keys[i].toArray();
            org.opencv.core.KeyPoint[] kb = keys[i + 1].toArray();
            double[] ra = VideoStreamingServer.cameraToVehicle(
                    VideoStreamingServer.CAM_YAW_DEG[i],
                    VideoStreamingServer.CAM_PITCH_DEG[i],
                    VideoStreamingServer.CAM_ROLL_DEG[i]);
            double[] rb = VideoStreamingServer.cameraToVehicle(
                    VideoStreamingServer.CAM_YAW_DEG[i + 1],
                    VideoStreamingServer.CAM_PITCH_DEG[i + 1],
                    VideoStreamingServer.CAM_ROLL_DEG[i + 1]);
            for (MatOfDMatch pair : knn) {
                org.opencv.core.DMatch[] dm = pair.toArray();
                if (dm.length < 2 || dm[0].distance > 0.75 * dm[1].distance) {
                    continue;
                }
                double[] rayA = rayAt(frames[i], i, ka[dm[0].queryIdx].pt.x, ka[dm[0].queryIdx].pt.y);
                double[] rayB = rayAt(frames[i + 1], i + 1, kb[dm[0].trainIdx].pt.x, kb[dm[0].trainIdx].pt.y);
                if (rayA == null || rayB == null) {
                    continue;
                }
                if (angleDeg(mul(ra, rayA), mul(rb, rayB)) > 40) {
                    continue;
                }
                out.add(new RayMatch(i, i + 1, rayA, rayB));
            }
        }
        for (int i = 0; i < 4; i++) {
            gray[i].release();
            keys[i].release();
            desc[i].release();
        }
        return out;
    }

    private static double[] rayAt(Mat frame, int cam, double u, double v) {
        CalibrationManager.CameraModel model = CalibrationManager.load(VideoStreamingServer.CAM_ROLE[cam]);
        double f = VideoStreamingServer.fisheyeFocal(frame.cols(), frame.rows(),
                VideoStreamingServer.INPUT_FISHEYE_FOV_DEG);
        double[] k = (model != null && model.hasIntrinsics)
                ? model.scaledK(frame.cols(), frame.rows())
                : new double[] {f, f, frame.cols() * 0.5, frame.rows() * 0.5};
        double k1 = model != null && model.hasIntrinsics ? model.k1 : 0;
        double k2 = model != null && model.hasIntrinsics ? model.k2 : 0;
        double k3 = model != null && model.hasIntrinsics ? model.k3 : 0;
        double k4 = model != null && model.hasIntrinsics ? model.k4 : 0;
        return CalibrationManager.unprojectFisheye(u, v, k, k1, k2, k3, k4);
    }

    /**
     * Solve each camera's rotation from ray pairs. Front yaw/pitch/roll stay the
     * reference; left is solved against front, then right, then rear.
     */
    private static void ransacExtrinsics(List<RayMatch> matches,
                                         double[] yaw, double[] pitch, double[] roll) {
        int[] solve = {0, 2, 3};
        int[] fixed = {1, 1, 2};
        for (int pair = 0; pair < solve.length; pair++) {
            int moving = solve[pair];
            int anchor = fixed[pair];
            List<RayMatch> subset = new ArrayList<>();
            for (RayMatch m : matches) {
                if ((m.a == anchor && m.b == moving) || (m.a == moving && m.b == anchor)) {
                    subset.add(m);
                }
            }
            System.out.println("  pair " + VideoStreamingServer.CAM_ROLE[anchor]
                    + "–" + VideoStreamingServer.CAM_ROLE[moving] + " candidates " + subset.size());
            if (subset.size() < 8) {
                continue;
            }
            double[] rAnchor = VideoStreamingServer.cameraToVehicle(
                    yaw[anchor], pitch[anchor], roll[anchor]);
            double bestInliers = 0;
            double[] best = {yaw[moving], pitch[moving], roll[moving]};
            java.util.Random rnd = new java.util.Random(7 + moving);
            for (int iter = 0; iter < 180; iter++) {
                List<double[]> src = new ArrayList<>();
                List<double[]> dst = new ArrayList<>();
                for (int s = 0; s < 3; s++) {
                    RayMatch m = subset.get(rnd.nextInt(subset.size()));
                    addKabschPair(m, moving, rAnchor, src, dst);
                }
                double[] r = kabsch(src, dst);
                if (r == null) {
                    continue;
                }
                double[] euler = eulerFromRotation(r);
                if (Math.abs(euler[0] - VideoStreamingServer.CAM_YAW_DEG[moving]) > 10
                        || Math.abs(euler[1] - VideoStreamingServer.CAM_PITCH_DEG[moving]) > 8
                        || Math.abs(euler[2] - VideoStreamingServer.CAM_ROLL_DEG[moving]) > 6) {
                    continue;
                }
                int inliers = countInliers(subset, moving, rAnchor, r, 3.0);
                if (inliers > bestInliers) {
                    bestInliers = inliers;
                    best = euler;
                }
            }
            if (bestInliers >= 8) {
                double[] rBest = VideoStreamingServer.cameraToVehicle(best[0], best[1], best[2]);
                List<double[]> src = new ArrayList<>();
                List<double[]> dst = new ArrayList<>();
                for (RayMatch m : subset) {
                    double[] rayMoving = m.a == moving ? m.rayA : m.rayB;
                    double[] rayAnchor = m.a == moving ? m.rayB : m.rayA;
                    double[] target = mul(rAnchor, rayAnchor);
                    if (angleDeg(mul(rBest, rayMoving), target) <= 3.0) {
                        src.add(rayMoving);
                        dst.add(target);
                    }
                }
                double[] refined = kabsch(src, dst);
                if (refined != null) {
                    best = eulerFromRotation(refined);
                }
                yaw[moving] = best[0];
                pitch[moving] = best[1];
                roll[moving] = best[2];
                System.out.printf(Locale.US,
                        "    %s inliers %.0f  yaw=%.2f pitch=%.2f roll=%.2f%n",
                        VideoStreamingServer.CAM_ROLE[moving], bestInliers,
                        best[0], best[1], best[2]);
            }
        }
    }

    private static void addKabschPair(RayMatch m, int moving, double[] rAnchor,
                                      List<double[]> src, List<double[]> dst) {
        double[] rayMoving = m.a == moving ? m.rayA : m.rayB;
        double[] rayAnchor = m.a == moving ? m.rayB : m.rayA;
        src.add(rayMoving);
        dst.add(mul(rAnchor, rayAnchor));
    }

    private static int countInliers(List<RayMatch> subset, int moving, double[] rAnchor,
                                    double[] rMoving, double threshDeg) {
        int n = 0;
        for (RayMatch m : subset) {
            double[] rayMoving = m.a == moving ? m.rayA : m.rayB;
            double[] rayAnchor = m.a == moving ? m.rayB : m.rayA;
            if (angleDeg(mul(rMoving, rayMoving), mul(rAnchor, rayAnchor)) <= threshDeg) {
                n++;
            }
        }
        return n;
    }

    private static void refineExtrinsics(List<RayMatch> matches,
                                         double[] yaw, double[] pitch, double[] roll) {
        double before = poseCost(matches, yaw, pitch, roll);
        for (int cam = 0; cam < 4; cam++) {
            if (cam == 1) {
                continue;
            }
            double baseY = yaw[cam];
            double baseP = pitch[cam];
            double baseR = roll[cam];
            double best = before;
            double by = baseY;
            double bp = baseP;
            double br = baseR;
            for (double dy = -4; dy <= 4; dy += 1) {
                for (double dp = -4; dp <= 4; dp += 1) {
                    for (double dr = -3; dr <= 3; dr += 1) {
                        yaw[cam] = baseY + dy;
                        pitch[cam] = baseP + dp;
                        roll[cam] = baseR + dr;
                        double score = poseCost(matches, yaw, pitch, roll);
                        if (score < best) {
                            best = score;
                            by = yaw[cam];
                            bp = pitch[cam];
                            br = roll[cam];
                        }
                    }
                }
            }
            yaw[cam] = by;
            pitch[cam] = bp;
            roll[cam] = br;
            before = best;
        }
        System.out.printf(Locale.US, "  refined ray error %.3f deg%n", before);
    }

    private static void optimizeGroundLines(List<Chain>[] chains, Mat[] frames,
                                            double[] yaw, double[] pitch, double[] roll) {
        for (int cam = 0; cam < 4; cam++) {
            if (chains[cam] == null || frames[cam] == null) {
                continue;
            }
            List<Chain> ground = new ArrayList<>();
            for (Chain c : chains[cam]) {
                if (c.pts.isEmpty()) {
                    continue;
                }
                double[] mid = c.pts.get(c.pts.size() / 2);
                if (mid[1] > 0.42 * c.imageHeight) {
                    ground.add(c);
                }
            }
            if (ground.size() < 2) {
                System.out.println("  " + VideoStreamingServer.CAM_ROLE[cam]
                        + ": not enough ground lines");
                continue;
            }
            double baseP = pitch[cam];
            double baseR = roll[cam];
            double best = groundCost(frames[cam], cam, ground, yaw[cam], baseP, baseR);
            double bp = baseP;
            double br = baseR;
            for (double dp = -8; dp <= 8; dp += 1) {
                for (double dr = -5; dr <= 5; dr += 1) {
                    double score = groundCost(frames[cam], cam, ground, yaw[cam], baseP + dp, baseR + dr);
                    if (score < best) {
                        best = score;
                        bp = baseP + dp;
                        br = baseR + dr;
                    }
                }
            }
            pitch[cam] = bp;
            roll[cam] = br;
            System.out.printf(Locale.US, "  %s ground pitch=%.2f roll=%.2f  scatter=%.4f%n",
                    VideoStreamingServer.CAM_ROLE[cam], bp, br, best);
        }
    }

    /** Colinearity of each ground line after it is intersected with Z = -1. */
    private static double groundCost(Mat frame, int cam, List<Chain> ground,
                                     double yaw, double pitch, double roll) {
        double[] r = VideoStreamingServer.cameraToVehicle(yaw, pitch, roll);
        double sum = 0;
        int used = 0;
        for (Chain chain : ground) {
            double mx = 0;
            double my = 0;
            int n = 0;
            List<double[]> xy = new ArrayList<>();
            for (double[] px : chain.pts) {
                double[] ray = rayAt(frame, cam, px[0], px[1]);
                if (ray == null) {
                    continue;
                }
                double[] v = mul(r, ray);
                if (v[2] >= -0.02) {
                    continue;
                }
                double t = -1.0 / v[2];
                double x = t * v[0];
                double y = t * v[1];
                xy.add(new double[] {x, y});
                mx += x;
                my += y;
                n++;
            }
            if (n < 6) {
                continue;
            }
            mx /= n;
            my /= n;
            double sxx = 0;
            double syy = 0;
            double sxy = 0;
            for (double[] p : xy) {
                double dx = p[0] - mx;
                double dy = p[1] - my;
                sxx += dx * dx;
                syy += dy * dy;
                sxy += dx * dy;
            }
            double half = 0.5 * (sxx + syy);
            double diff = 0.5 * (sxx - syy);
            double lam = half - Math.sqrt(diff * diff + sxy * sxy);
            sum += lam / n;
            used++;
        }
        return used == 0 ? 1e6 : sum / used;
    }

    private static double poseCost(List<RayMatch> matches, double[] yaw, double[] pitch, double[] roll) {
        double[][] r = new double[4][];
        for (int i = 0; i < 4; i++) {
            r[i] = VideoStreamingServer.cameraToVehicle(yaw[i], pitch[i], roll[i]);
        }
        double sum = 0;
        int n = 0;
        for (RayMatch m : matches) {
            sum += Math.min(15.0, angleDeg(mul(r[m.a], m.rayA), mul(r[m.b], m.rayB)));
            n++;
        }
        return n == 0 ? 1e6 : sum / n;
    }

    private static double[] kabsch(List<double[]> src, List<double[]> dst) {
        if (src.size() < 2 || src.size() != dst.size()) {
            return null;
        }
        Mat h = Mat.zeros(3, 3, CvType.CV_64F);
        for (int i = 0; i < src.size(); i++) {
            double[] s = src.get(i);
            double[] d = dst.get(i);
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    double[] cur = h.get(r, c);
                    h.put(r, c, cur[0] + d[r] * s[c]);
                }
            }
        }
        Mat w = new Mat();
        Mat u = new Mat();
        Mat vt = new Mat();
        Core.SVDecomp(h, w, u, vt);
        Mat r = new Mat();
        Core.gemm(u, vt, 1, new Mat(), 0, r, 0);
        if (Core.determinant(r) < 0) {
            u.put(0, 2, -u.get(0, 2)[0]);
            u.put(1, 2, -u.get(1, 2)[0]);
            u.put(2, 2, -u.get(2, 2)[0]);
            Core.gemm(u, vt, 1, new Mat(), 0, r, 0);
        }
        double[] out = new double[9];
        r.get(0, 0, out);
        h.release();
        w.release();
        u.release();
        vt.release();
        r.release();
        return out;
    }

    /** Inverse of {@link VideoStreamingServer#cameraToVehicle}. */
    static double[] eulerFromRotation(double[] cameraToVehicle) {
        double[] rcvT = {
                0, 1, 0,
                0, 0, -1,
                1, 0, 0
        };
        double[] m = mul3(cameraToVehicle, rcvT);
        double beta = Math.atan2(-m[6], Math.hypot(m[0], m[3]));
        double yaw = Math.atan2(m[3], m[0]);
        double roll = Math.atan2(m[7], m[8]);
        return new double[] {
                Math.toDegrees(yaw),
                Math.toDegrees(-beta),
                Math.toDegrees(roll)
        };
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double[] mul(double[] r, double[] v) {
        return new double[] {
                r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
                r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
                r[6] * v[0] + r[7] * v[1] + r[8] * v[2]
        };
    }

    private static double[] mul3(double[] a, double[] b) {
        double[] c = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                c[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
            }
        }
        return c;
    }

    private static double angleDeg(double[] a, double[] b) {
        double na = Math.hypot(a[0], Math.hypot(a[1], a[2]));
        double nb = Math.hypot(b[0], Math.hypot(b[1], b[2]));
        if (na < 1e-9 || nb < 1e-9) {
            return 180;
        }
        double dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
        if (dot > 1) {
            dot = 1;
        }
        if (dot < -1) {
            dot = -1;
        }
        return Math.toDegrees(Math.acos(dot));
    }

    private interface Scorer {
        double score(double[] p);
    }

    private static double[] nelderMead(double[] x0, double[] step, int iterations, Scorer scorer) {
        int n = x0.length;
        double[][] simplex = new double[n + 1][];
        double[] scores = new double[n + 1];
        simplex[0] = x0.clone();
        scores[0] = scorer.score(simplex[0]);
        for (int i = 0; i < n; i++) {
            simplex[i + 1] = x0.clone();
            simplex[i + 1][i] += step[i];
            scores[i + 1] = scorer.score(simplex[i + 1]);
        }
        for (int iter = 0; iter < iterations; iter++) {
            sortSimplex(simplex, scores);
            double[] centroid = new double[n];
            for (int i = 0; i < n; i++) {
                for (int d = 0; d < n; d++) {
                    centroid[d] += simplex[i][d];
                }
            }
            for (int d = 0; d < n; d++) {
                centroid[d] /= n;
            }
            double[] worst = simplex[n];
            double[] reflected = new double[n];
            for (int d = 0; d < n; d++) {
                reflected[d] = centroid[d] + (centroid[d] - worst[d]);
            }
            double fr = scorer.score(reflected);
            if (fr < scores[0]) {
                double[] expanded = new double[n];
                for (int d = 0; d < n; d++) {
                    expanded[d] = centroid[d] + 2 * (reflected[d] - centroid[d]);
                }
                double fe = scorer.score(expanded);
                if (fe < fr) {
                    simplex[n] = expanded;
                    scores[n] = fe;
                } else {
                    simplex[n] = reflected;
                    scores[n] = fr;
                }
            } else if (fr < scores[n - 1]) {
                simplex[n] = reflected;
                scores[n] = fr;
            } else {
                double[] contracted = new double[n];
                double[] toward = fr < scores[n] ? reflected : worst;
                for (int d = 0; d < n; d++) {
                    contracted[d] = centroid[d] + 0.5 * (toward[d] - centroid[d]);
                }
                double fc = scorer.score(contracted);
                if (fc < Math.min(fr, scores[n])) {
                    simplex[n] = contracted;
                    scores[n] = fc;
                } else {
                    for (int i = 1; i <= n; i++) {
                        for (int d = 0; d < n; d++) {
                            simplex[i][d] = simplex[0][d] + 0.5 * (simplex[i][d] - simplex[0][d]);
                        }
                        scores[i] = scorer.score(simplex[i]);
                    }
                }
            }
        }
        sortSimplex(simplex, scores);
        return simplex[0];
    }

    private static void sortSimplex(double[][] simplex, double[] scores) {
        for (int i = 0; i < scores.length; i++) {
            for (int j = i + 1; j < scores.length; j++) {
                if (scores[j] < scores[i]) {
                    double tmp = scores[i];
                    scores[i] = scores[j];
                    scores[j] = tmp;
                    double[] row = simplex[i];
                    simplex[i] = simplex[j];
                    simplex[j] = row;
                }
            }
        }
    }

    /** Synthetic grid with a known fisheye model. Returns 0 when the fit beats D=0. */
    static int selfTest() {
        double[] r = VideoStreamingServer.cameraToVehicle(90, -14, 3);
        double[] euler = eulerFromRotation(r);
        if (Math.abs(euler[0] - 90) > 0.5 || Math.abs(euler[1] + 14) > 0.5
                || Math.abs(euler[2] - 3) > 0.5) {
            System.err.printf(Locale.US, "Euler roundtrip failed: %.2f %.2f %.2f%n",
                    euler[0], euler[1], euler[2]);
            return 1;
        }
        int w = 960;
        int h = 720;
        double f = VideoStreamingServer.fisheyeFocal(w, h, 170);
        double cx = w * 0.51;
        double cy = h * 0.48;
        double k1 = 0.1;
        double k2 = -0.04;
        Mat img = new Mat(h, w, CvType.CV_8UC3, new Scalar(240, 240, 240));
        double[] k = {f, f, cx, cy};
        for (int i = -4; i <= 4; i++) {
            drawWorldLine(img, k, k1, k2, i * 0.18, -0.7, i * 0.18, 0.7);
            drawWorldLine(img, k, k1, k2, -0.75, i * 0.14, 0.75, i * 0.14);
        }
        List<Mat> frames = new ArrayList<>();
        frames.add(img);
        Fit fit = fitCamera(frames, w, h);
        img.release();
        System.out.printf(Locale.US,
                "self-test truth f=%.2f k1=%.3f  fit f=%.2f k1=%.3f accepted=%s lines=%d%n",
                f, k1, fit.f, fit.k1, fit.accepted, fit.chains.size());
        if (!fit.accepted || fit.chains.size() < 4) {
            System.err.println("self-test: line fit did not improve on the known grid");
            return 1;
        }
        if (Math.abs(fit.f - f) / f > 0.2) {
            System.err.println("self-test: focal length off by more than 20%");
            return 1;
        }
        System.out.println("self-test OK");
        return 0;
    }

    private static void drawWorldLine(Mat img, double[] k, double k1, double k2,
                                      double x0, double y0, double x1, double y1) {
        Point prev = null;
        for (int i = 0; i <= 40; i++) {
            double t = i / 40.0;
            double x = x0 + (x1 - x0) * t;
            double y = y0 + (y1 - y0) * t;
            float[] uv = CalibrationManager.projectFisheye(x, y, 1.0, k, k1, k2, 0, 0);
            if (uv == null) {
                prev = null;
                continue;
            }
            Point cur = new Point(uv[0], uv[1]);
            if (prev != null) {
                Imgproc.line(img, prev, cur, new Scalar(10, 10, 10), 2);
            }
            prev = cur;
        }
    }
}
