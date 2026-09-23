import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

/**
 * Offline chessboard calibration for the surround fisheye cameras.
 *
 * Run once (or whenever the lenses / mounts change):
 *   java VideoStreamingServer --calibrate
 *
 * Production start loads {@code calib/<role>.json} if present and never
 * re-detects chessboards on live frames.
 *
 * OpenCV fisheye flags are passed as numeric values because the Java
 * bindings share names with the pinhole Calib3d constants.
 *   1  USE_INTRINSIC_GUESS
 *   2  RECOMPUTE_EXTRINSIC
 *   8  FIX_SKEW
 */
public final class CalibrationManager {

    static final String CALIB_DIR = "calib";
    private static final Map<String, CameraModel> LOADED = new HashMap<>();

    /** Inner corners (columns × rows), not squares. */
    static final int DEFAULT_PATTERN_COLS = 9;
    static final int DEFAULT_PATTERN_ROWS = 6;
    static final double DEFAULT_SQUARE_M = 0.030;
    static final int DEFAULT_MIN_VIEWS = 12;
    static final int DEFAULT_MAX_VIEWS = 40;
    static final int DEFAULT_FRAME_STEP = 5;
    /** Reject chessboard detections smaller than this fraction of image diagonal. */
    private static final double MIN_CORNER_SPAN_FRAC = 0.10;
    /** Drop calibration views above this multiple of the median reprojection error. */
    private static final double OUTLIER_RMS_FACTOR = 2.4;
    private static final double OUTLIER_RMS_FLOOR_PX = 1.15;

    private CalibrationManager() {}

    static final class Options {
        int patternCols = DEFAULT_PATTERN_COLS;
        int patternRows = DEFAULT_PATTERN_ROWS;
        double squareMeters = DEFAULT_SQUARE_M;
        int minViews = DEFAULT_MIN_VIEWS;
        int maxViews = DEFAULT_MAX_VIEWS;
        int frameStep = DEFAULT_FRAME_STEP;
        String onlyRole = null;
        Path folder = Paths.get(".").toAbsolutePath().normalize();
    }

    static final class CameraModel {
        final String role;
        final int imageWidth;
        final int imageHeight;
        final double rms;
        final double fx;
        final double fy;
        final double cx;
        final double cy;
        final double k1;
        final double k2;
        final double k3;
        final double k4;
        final boolean hasIntrinsics;
        final boolean hasExtrinsics;
        final double yawDeg;
        final double pitchDeg;
        final double rollDeg;
        final double[] rvec;
        final double[] tvec;

        CameraModel(String role, int imageWidth, int imageHeight, double rms,
                    double fx, double fy, double cx, double cy,
                    double k1, double k2, double k3, double k4,
                    boolean hasIntrinsics, boolean hasExtrinsics,
                    double yawDeg, double pitchDeg, double rollDeg,
                    double[] rvec, double[] tvec) {
            this.role = role;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.rms = rms;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.k1 = k1;
            this.k2 = k2;
            this.k3 = k3;
            this.k4 = k4;
            this.hasIntrinsics = hasIntrinsics;
            this.hasExtrinsics = hasExtrinsics;
            this.yawDeg = yawDeg;
            this.pitchDeg = pitchDeg;
            this.rollDeg = rollDeg;
            this.rvec = rvec;
            this.tvec = tvec;
        }

        double[] scaledK(int srcW, int srcH) {
            double sx = imageWidth > 0 ? (double) srcW / imageWidth : 1.0;
            double sy = imageHeight > 0 ? (double) srcH / imageHeight : 1.0;
            return new double[] { fx * sx, fy * sy, cx * sx, cy * sy };
        }

        CameraModel withMountPose(double yawDeg, double pitchDeg, double rollDeg,
                                  boolean hasExtrinsics) {
            return new CameraModel(role, imageWidth, imageHeight, rms,
                    fx, fy, cx, cy, k1, k2, k3, k4,
                    hasIntrinsics, hasExtrinsics,
                    yawDeg, pitchDeg, rollDeg, rvec, tvec);
        }
    }

    /**
     * Yaw / pitch / roll (degrees) used when building the spherical remap.
     * Intrinsics alone do not change mount angles; {@code hasExtrinsics} must be
     * true (chessboard ground pose or {@code --align-mounts}) for saved pitch/roll.
     */
    static double[] mountDegreesForStitch(int camIndex, CameraModel calib,
                                         double pitchOverride, double rollOverride) {
        double yaw = VideoStreamingServer.CAM_YAW_DEG[camIndex];
        double pitch = VideoStreamingServer.CAM_PITCH_DEG[camIndex];
        double roll = VideoStreamingServer.CAM_ROLL_DEG[camIndex];
        if (calib != null && calib.hasExtrinsics) {
            yaw = calib.yawDeg;
            pitch = calib.pitchDeg;
            roll = calib.rollDeg;
        }
        if (!Double.isNaN(pitchOverride)) {
            pitch = pitchOverride;
        }
        if (!Double.isNaN(rollOverride)) {
            roll = rollOverride;
        }
        return new double[] { yaw, pitch, roll };
    }

    static boolean saveMountPose(String role, double yawDeg, double pitchDeg,
                                 double rollDeg) {
        CameraModel existing = load(role);
        if (existing == null || !existing.hasIntrinsics) {
            System.err.println("Cannot save mount pose for '" + role
                    + "': run --calibrate first (intrinsics required).");
            return false;
        }
        CameraModel updated = existing.withMountPose(
                yawDeg, pitchDeg, rollDeg, true);
        return save(updated);
    }

    static Path calibFile(String role) {
        return Paths.get(CALIB_DIR).resolve(role.toLowerCase(Locale.ROOT) + ".json");
    }

    static CameraModel load(String role) {
        String key = role.toLowerCase(Locale.ROOT);
        if (LOADED.containsKey(key)) {
            return LOADED.get(key);
        }
        Path file = calibFile(key);
        if (!Files.isRegularFile(file)) {
            LOADED.put(key, null);
            return null;
        }
        try {
            String json = Files.readString(file, StandardCharsets.UTF_8);
            CameraModel model = parseJson(key, json);
            LOADED.put(key, model);
            return model;
        } catch (IOException e) {
            System.err.println("Could not read " + file + ": " + e.getMessage());
            LOADED.put(key, null);
            return null;
        } catch (RuntimeException e) {
            System.err.println("Invalid calibration file " + file + ": " + e.getMessage());
            LOADED.put(key, null);
            return null;
        }
    }

    static int run(Options opt) {
        Path dir = Paths.get(CALIB_DIR);
        try {
            Files.createDirectories(dir);
        } catch (IOException e) {
            System.err.println("Cannot create " + dir + ": " + e.getMessage());
            return 1;
        }

        System.out.println("CALIBRATION MODE");
        System.out.println("Pattern: " + opt.patternCols + " × " + opt.patternRows
                + " inner corners");
        System.out.println("Square size: " + (opt.squareMeters * 1000.0) + " mm");
        System.out.println("Target views: " + opt.minViews + "–" + opt.maxViews);
        System.out.println("Working folder: " + opt.folder);
        System.out.println();

        int ok = 0;
        int fail = 0;
        for (int i = 0; i < VideoStreamingServer.CAM_ROLE.length; i++) {
            String role = VideoStreamingServer.CAM_ROLE[i];
            if (opt.onlyRole != null && !opt.onlyRole.equalsIgnoreCase(role)) {
                continue;
            }
            System.out.println("── " + role.toUpperCase(Locale.ROOT) + " ──");
            boolean saved = calibrateRole(role, i, opt);
            if (saved) {
                ok++;
            } else {
                fail++;
            }
            System.out.println();
        }

        System.out.println("Calibration finished. saved=" + ok + " skipped=" + fail);
        if (ok == 0) {
            System.err.println("No camera was calibrated. Place a printed chessboard in");
            System.err.println("front of each installed camera, or drop images/video into:");
            System.err.println("  " + dir.toAbsolutePath() + "/<left|front|right|rear>/");
            System.err.println("Then re-run with --calibrate. Production stitching will keep");
            System.err.println("using the equidistant FOV fallback until files exist.");
            return 2;
        }
        return 0;
    }

    static boolean calibrateRole(String role, int camIndex, Options opt) {
        Size pattern = new Size(opt.patternCols, opt.patternRows);
        Mat objectTemplate = chessboardObjectPoints(opt.patternCols, opt.patternRows, opt.squareMeters);

        List<Mat> objectPoints = new ArrayList<>();
        List<Mat> imagePoints = new ArrayList<>();
        int[] imageSize = { 0, 0 };

        collectFromImageFolder(opt.folder.resolve(CALIB_DIR).resolve(role),
                pattern, objectTemplate, objectPoints, imagePoints, imageSize, opt.maxViews);
        if (imagePoints.size() < opt.maxViews) {
            collectFromNamedImages(opt.folder.resolve(CALIB_DIR), role, pattern,
                    objectTemplate, objectPoints, imagePoints, imageSize, opt.maxViews);
        }
        if (imagePoints.size() < opt.minViews) {
            collectFromVideos(role, camIndex, opt, pattern, objectTemplate,
                    objectPoints, imagePoints, imageSize);
        }

        int captured = imagePoints.size();
        System.out.println("Captured: " + captured + "/" + opt.maxViews);
        if (captured < opt.minViews) {
            System.err.println("Need at least " + opt.minViews
                    + " diverse chessboard views for '" + role + "' (got " + captured + ").");
            releaseAll(objectPoints);
            releaseAll(imagePoints);
            objectTemplate.release();
            return false;
        }

        int w = imageSize[0];
        int h = imageSize[1];
        System.out.println("Camera resolution: " + w + " × " + h);
        System.out.println("Calibrating...");

        FisheyeCalibResult cal = runRefinedFisheyeCalibrate(
                role, objectPoints, imagePoints, w, h, opt.minViews);
        if (cal == null) {
            releaseAll(objectPoints);
            releaseAll(imagePoints);
            objectTemplate.release();
            return false;
        }
        Mat K = cal.K;
        Mat D = cal.D;
        List<Mat> rvecs = cal.rvecs;
        List<Mat> tvecs = cal.tvecs;
        double rms = cal.rms;
        if (cal.droppedViews > 0) {
            System.out.println("Refinement: kept " + imagePoints.size() + " views after dropping "
                    + cal.droppedViews + " high-error outlier(s).");
        }

        double fx = K.get(0, 0)[0];
        double fy = K.get(1, 1)[0];
        double cx = K.get(0, 2)[0];
        double cy = K.get(1, 2)[0];
        double k1 = getCoeff(D, 0);
        double k2 = getCoeff(D, 1);
        double k3 = getCoeff(D, 2);
        double k4 = getCoeff(D, 3);

        PoseEstimate pose = estimateVehicleExtrinsics(
                rvecs, tvecs, imagePoints, camIndex);

        System.out.printf(Locale.US, "RMS error: %.4f pixels%n", rms);
        System.out.println("K =");
        System.out.printf(Locale.US, "[ %.4f   0   %.4f ]%n", fx, cx);
        System.out.printf(Locale.US, "[ 0   %.4f   %.4f ]%n", fy, cy);
        System.out.println("[ 0    0    1 ]");
        System.out.println("D =");
        System.out.printf(Locale.US, "[ %.6f  %.6f  %.6f  %.6f ]%n", k1, k2, k3, k4);
        if (pose.hasExtrinsics) {
            System.out.printf(Locale.US,
                    "Extrinsics (board→camera, ground-board assumption):%n"
                            + "  yaw=%.2f  pitch=%.2f  roll=%.2f  t=[%.3f %.3f %.3f]%n",
                    pose.yawDeg, pose.pitchDeg, pose.rollDeg,
                    pose.tvec[0], pose.tvec[1], pose.tvec[2]);
        } else {
            System.out.println("Extrinsics: keeping configured vehicle yaw/pitch/roll"
                    + " (chessboard pose was not stable enough).");
            System.out.println("  Seam alignment still off? After --calibrate run:");
            System.out.println("    java VideoStreamingServer --align-mounts");
            System.out.println("  (uses your left/front/right/rear clips, no checkerboard).");
        }
        System.out.println("Generating remapping LUT at runtime from K + D.");

        CameraModel model = new CameraModel(
                role, w, h, rms, fx, fy, cx, cy, k1, k2, k3, k4,
                true, pose.hasExtrinsics,
                pose.yawDeg, pose.pitchDeg, pose.rollDeg,
                pose.rvec, pose.tvec);
        boolean saved = save(model);
        if (saved) {
            System.out.println("Calibration saved: " + calibFile(role).toAbsolutePath());
        }

        releaseAll(objectPoints);
        releaseAll(imagePoints);
        releaseAll(rvecs);
        releaseAll(tvecs);
        objectTemplate.release();
        K.release();
        D.release();
        return saved;
    }

    static boolean save(CameraModel m) {
        String json = toJson(m);
        Path file = calibFile(m.role);
        try {
            Files.createDirectories(file.getParent());
            Files.writeString(file, json, StandardCharsets.UTF_8);
            LOADED.put(m.role.toLowerCase(Locale.ROOT), m);
            return true;
        } catch (IOException e) {
            System.err.println("Could not write " + file + ": " + e.getMessage());
            return false;
        }
    }

    /**
     * Kannala-Brandt fisheye projection of a camera-frame ray into pixels.
     * {@code k} is {fx, fy, cx, cy}. Returns {u, v} or null if behind the camera.
     */
    static float[] projectFisheye(double xc, double yc, double zc,
                                  double[] k, double k1, double k2, double k3, double k4) {
        if (zc <= 1e-4) {
            return null;
        }
        double xn = xc / zc;
        double yn = yc / zc;
        double r = Math.hypot(xn, yn);
        double theta = Math.atan(r);
        double t2 = theta * theta;
        double t4 = t2 * t2;
        double t6 = t4 * t2;
        double t8 = t4 * t4;
        double thetaD = theta * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
        double scale = r > 1e-12 ? thetaD / r : 1.0;
        double xd = xn * scale;
        double yd = yn * scale;
        return new float[] {
            (float) (k[0] * xd + k[2]),
            (float) (k[1] * yd + k[3])
        };
    }

    /**
     * Inverse of {@link #projectFisheye}: pixel → unit ray in the OpenCV camera frame.
     */
    static double[] unprojectFisheye(double u, double v, double[] k,
                                     double k1, double k2, double k3, double k4) {
        if (k == null || k.length < 4 || Math.abs(k[0]) < 1e-6 || Math.abs(k[1]) < 1e-6) {
            return null;
        }
        double xd = (u - k[2]) / k[0];
        double yd = (v - k[3]) / k[1];
        double rd = Math.hypot(xd, yd);
        double theta = rd;
        for (int iter = 0; iter < 10; iter++) {
            double t2 = theta * theta;
            double t4 = t2 * t2;
            double t6 = t4 * t2;
            double t8 = t4 * t4;
            double td = theta * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
            double dt = 1.0 + 3.0 * k1 * t2 + 5.0 * k2 * t4 + 7.0 * k3 * t6 + 9.0 * k4 * t8;
            if (Math.abs(dt) < 1e-9) {
                break;
            }
            theta -= (td - rd) / dt;
            if (theta < 0) {
                theta = 0;
            }
            if (theta > Math.PI * 0.49) {
                theta = Math.PI * 0.49;
            }
        }
        double az = Math.atan2(yd, xd);
        double st = Math.sin(theta);
        return new double[] { st * Math.cos(az), st * Math.sin(az), Math.cos(theta) };
    }

    private static void collectFromImageFolder(Path folder, Size pattern, Mat objectTemplate,
                                               List<Mat> objectPoints, List<Mat> imagePoints,
                                               int[] imageSize, int maxViews) {
        if (!Files.isDirectory(folder)) {
            return;
        }
        List<Path> files = new ArrayList<>();
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(folder)) {
            for (Path p : stream) {
                if (isImage(p)) {
                    files.add(p);
                }
            }
        } catch (IOException ignored) {
            return;
        }
        files.sort((a, b) -> a.getFileName().toString().compareToIgnoreCase(
                b.getFileName().toString()));
        for (Path p : files) {
            if (imagePoints.size() >= maxViews) {
                break;
            }
            Mat bgr = Imgcodecs.imread(p.toAbsolutePath().toString());
            if (bgr == null || bgr.empty()) {
                continue;
            }
            tryAddView(bgr, pattern, objectTemplate, objectPoints, imagePoints, imageSize,
                    "image " + p.getFileName());
            bgr.release();
        }
    }

    private static void collectFromNamedImages(Path calibRoot, String role, Size pattern,
                                               Mat objectTemplate, List<Mat> objectPoints,
                                               List<Mat> imagePoints, int[] imageSize,
                                               int maxViews) {
        if (!Files.isDirectory(calibRoot)) {
            return;
        }
        String prefix = role.toLowerCase(Locale.ROOT);
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(calibRoot)) {
            for (Path p : stream) {
                if (imagePoints.size() >= maxViews) {
                    break;
                }
                if (!isImage(p)) {
                    continue;
                }
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                if (!n.startsWith(prefix)) {
                    continue;
                }
                Mat bgr = Imgcodecs.imread(p.toAbsolutePath().toString());
                if (bgr == null || bgr.empty()) {
                    continue;
                }
                tryAddView(bgr, pattern, objectTemplate, objectPoints, imagePoints, imageSize,
                        "image " + p.getFileName());
                bgr.release();
            }
        } catch (IOException ignored) {
        }
    }

    private static void collectFromVideos(String role, int camIndex, Options opt,
                                          Size pattern, Mat objectTemplate,
                                          List<Mat> objectPoints, List<Mat> imagePoints,
                                          int[] imageSize) {
        List<Path> videos = new ArrayList<>();
        Path calibRoot = opt.folder.resolve(CALIB_DIR);
        addIfUsable(videos, calibRoot.resolve(role + ".mp4"));
        addIfUsable(videos, calibRoot.resolve(role + ".mov"));
        addIfUsable(videos, calibRoot.resolve(role + "_calib.mp4"));
        addIfUsable(videos, calibRoot.resolve(role + "_1.mp4"));
        Path production = VideoStreamingServer.findClip(opt.folder, role);
        if (production != null) {
            videos.add(production);
        }

        for (Path video : videos) {
            if (imagePoints.size() >= opt.maxViews) {
                break;
            }
            Path decodable = VideoStreamingServer.ensureDecodable(video);
            VideoCapture cap = VideoStreamingServer.openVideo(decodable);
            if (cap == null || !cap.isOpened()) {
                System.err.println("Could not open calibration video: " + video);
                continue;
            }
            System.out.println("Scanning " + decodable.getFileName() + " for chessboard views...");
            Mat frame = new Mat();
            int idx = 0;
            int accepted = imagePoints.size();
            while (imagePoints.size() < opt.maxViews) {
                if (!VideoStreamingServer.readBgr(cap, frame)) {
                    break;
                }
                if (idx % Math.max(1, opt.frameStep) == 0) {
                    tryAddView(frame, pattern, objectTemplate, objectPoints, imagePoints,
                            imageSize, "frame " + idx);
                }
                idx++;
            }
            cap.release();
            frame.release();
            System.out.println("  +" + (imagePoints.size() - accepted) + " views from video");
        }
    }

    private static boolean tryAddView(Mat bgr, Size pattern, Mat objectTemplate,
                                      List<Mat> objectPoints, List<Mat> imagePoints,
                                      int[] imageSize, String label) {
        MatOfPoint2f corners = detectChessboard(bgr, pattern);
        if (corners == null) {
            return false;
        }
        if (imageSize[0] == 0) {
            imageSize[0] = bgr.cols();
            imageSize[1] = bgr.rows();
        } else if (bgr.cols() != imageSize[0] || bgr.rows() != imageSize[1]) {
            System.err.println("Skipping " + label + ": resolution "
                    + bgr.cols() + "×" + bgr.rows() + " != "
                    + imageSize[0] + "×" + imageSize[1]);
            corners.release();
            return false;
        }
        Mat img64 = new Mat();
        corners.convertTo(img64, CvType.CV_64FC2);
        corners.release();

        double diag = Math.hypot(imageSize[0], imageSize[1]);
        double span = cornerSpan(img64);
        if (span < MIN_CORNER_SPAN_FRAC * diag) {
            System.err.println("Skipping " + label + ": board too small in frame ("
                    + String.format(Locale.US, "%.0f", span) + " px span).");
            img64.release();
            return false;
        }
        Mat obj = objectTemplate.clone();
        objectPoints.add(obj);
        imagePoints.add(img64);
        System.out.println("Captured: " + imagePoints.size() + "  (" + label + ")");
        return true;
    }

    static MatOfPoint2f detectChessboard(Mat bgr, Size pattern) {
        Mat gray = toGray(bgr);
        MatOfPoint2f corners = detectOnGray(gray, pattern);
        if (corners == null) {
            Mat claheGray = applyClahe(gray);
            corners = detectOnGray(claheGray, pattern);
            claheGray.release();
        }
        if (corners == null) {
            gray.release();
            return null;
        }
        refineCorners(gray, corners);
        gray.release();
        return corners;
    }

    private static Mat toGray(Mat bgr) {
        Mat gray = new Mat();
        if (bgr.channels() == 1) {
            bgr.copyTo(gray);
        } else {
            Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
        }
        return gray;
    }

    private static Mat applyClahe(Mat gray) {
        Mat out = new Mat();
        try {
            org.opencv.imgproc.CLAHE clahe = Imgproc.createCLAHE(2.5, new Size(8, 8));
            clahe.apply(gray, out);
        } catch (Throwable t) {
            gray.copyTo(out);
        }
        return out;
    }

    private static MatOfPoint2f detectOnGray(Mat gray, Size pattern) {
        int[] flagSets = {
            Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE,
            Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE
                    + Calib3d.CALIB_CB_FAST_CHECK
        };
        for (int flags : flagSets) {
            MatOfPoint2f corners = new MatOfPoint2f();
            boolean found = Calib3d.findChessboardCorners(gray, pattern, corners, flags);
            if (found && corners.total() >= (long) (pattern.width * pattern.height)) {
                return corners;
            }
            corners.release();
        }
        try {
            MatOfPoint2f corners = new MatOfPoint2f();
            int sbFlags = Calib3d.CALIB_CB_EXHAUSTIVE + Calib3d.CALIB_CB_ACCURACY;
            boolean found = Calib3d.findChessboardCornersSB(gray, pattern, corners, sbFlags);
            if (found && corners.total() >= (long) (pattern.width * pattern.height)) {
                return corners;
            }
            corners.release();
            corners = new MatOfPoint2f();
            found = Calib3d.findChessboardCornersSB(gray, pattern, corners);
            if (found && corners.total() >= (long) (pattern.width * pattern.height)) {
                return corners;
            }
            corners.release();
        } catch (Throwable ignored) {
        }
        return null;
    }

    private static void refineCorners(Mat gray, MatOfPoint2f corners) {
        Imgproc.cornerSubPix(gray, corners, new Size(7, 7), new Size(-1, -1),
                new TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 60, 0.008));
    }

    private static final class FisheyeCalibResult {
        Mat K;
        Mat D;
        List<Mat> rvecs;
        List<Mat> tvecs;
        double rms;
        int droppedViews;
    }

    private static FisheyeCalibResult runRefinedFisheyeCalibrate(
            String role, List<Mat> objectPoints, List<Mat> imagePoints,
            int w, int h, int minViews) {
        Mat K = Mat.eye(3, 3, CvType.CV_64F);
        seedIntrinsicGuess(role, w, h, K);
        Mat D = Mat.zeros(4, 1, CvType.CV_64F);
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();

        int flags = Calib3d.fisheye_CALIB_USE_INTRINSIC_GUESS
                | Calib3d.fisheye_CALIB_RECOMPUTE_EXTRINSIC
                | Calib3d.fisheye_CALIB_FIX_SKEW;
        TermCriteria criteria = new TermCriteria(
                TermCriteria.COUNT + TermCriteria.EPS, 250, 1e-9);

        double rms;
        try {
            rms = Calib3d.fisheye_calibrate(
                    objectPoints, imagePoints, new Size(w, h),
                    K, D, rvecs, tvecs, flags, criteria);
        } catch (CvException e) {
            System.err.println("fisheye.calibrate failed: " + e.getMessage());
            releaseAll(rvecs);
            releaseAll(tvecs);
            rvecs = new ArrayList<>();
            tvecs = new ArrayList<>();
            seedEquidistantGuess(w, h, K);
            D = Mat.zeros(4, 1, CvType.CV_64F);
            try {
                rms = Calib3d.fisheye_calibrate(
                        objectPoints, imagePoints, new Size(w, h),
                        K, D, rvecs, tvecs, flags, criteria);
                System.out.println("Recovered with equidistant intrinsic guess.");
            } catch (CvException e2) {
                System.err.println("fisheye.calibrate retry failed: " + e2.getMessage());
                K.release();
                D.release();
                releaseAll(rvecs);
                releaseAll(tvecs);
                return null;
            }
        }

        int dropped = pruneOutlierViews(objectPoints, imagePoints, K, D, rvecs, tvecs,
                minViews);
        if (dropped > 0 && imagePoints.size() >= minViews) {
            releaseAll(rvecs);
            releaseAll(tvecs);
            rvecs = new ArrayList<>();
            tvecs = new ArrayList<>();
            try {
                rms = Calib3d.fisheye_calibrate(
                        objectPoints, imagePoints, new Size(w, h),
                        K, D, rvecs, tvecs, flags, criteria);
            } catch (CvException e) {
                System.err.println("fisheye.calibrate (refine pass) failed: " + e.getMessage());
                K.release();
                D.release();
                releaseAll(rvecs);
                releaseAll(tvecs);
                return null;
            }
        }

        double k3 = Math.abs(getCoeff(D, 2));
        double k4 = Math.abs(getCoeff(D, 3));
        if (k3 + k4 > 0.85 && imagePoints.size() >= minViews) {
            int stableFlags = flags | Calib3d.fisheye_CALIB_FIX_K3 | Calib3d.fisheye_CALIB_FIX_K4;
            releaseAll(rvecs);
            releaseAll(tvecs);
            rvecs = new ArrayList<>();
            tvecs = new ArrayList<>();
            try {
                rms = Calib3d.fisheye_calibrate(
                        objectPoints, imagePoints, new Size(w, h),
                        K, D, rvecs, tvecs, stableFlags, criteria);
                System.out.println("Stabilized high-order distortion (FIX_K3 | FIX_K4).");
            } catch (CvException e) {
                System.err.println("fisheye.calibrate (stable D) failed: " + e.getMessage());
            }
        }

        FisheyeCalibResult out = new FisheyeCalibResult();
        out.K = K;
        out.D = D;
        out.rvecs = rvecs;
        out.tvecs = tvecs;
        out.rms = rms;
        out.droppedViews = dropped;
        return out;
    }

    private static void seedIntrinsicGuess(String role, int w, int h, Mat K) {
        seedEquidistantGuess(w, h, K);
    }

    private static void seedEquidistantGuess(int w, int h, Mat K) {
        double fGuess = VideoStreamingServer.fisheyeFocal(w, h,
                VideoStreamingServer.INPUT_FISHEYE_FOV_DEG);
        K.put(0, 0, fGuess);
        K.put(1, 1, fGuess);
        K.put(0, 2, w * VideoStreamingServer.FISHEYE_CX);
        K.put(1, 2, h * VideoStreamingServer.FISHEYE_CY);
    }

    private static int pruneOutlierViews(List<Mat> objectPoints, List<Mat> imagePoints,
                                         Mat K, Mat D, List<Mat> rvecs, List<Mat> tvecs,
                                         int minViews) {
        if (objectPoints.size() != imagePoints.size()
                || rvecs.size() != tvecs.size()
                || objectPoints.size() < minViews + 1) {
            return 0;
        }
        List<Double> errs = new ArrayList<>();
        for (int i = 0; i < objectPoints.size(); i++) {
            errs.add(perViewRms(objectPoints.get(i), imagePoints.get(i),
                    rvecs.get(i), tvecs.get(i), K, D));
        }
        double median = median(errs);
        double limit = Math.max(OUTLIER_RMS_FLOOR_PX, median * OUTLIER_RMS_FACTOR);
        int dropped = 0;
        for (int i = objectPoints.size() - 1; i >= 0; i--) {
            if (objectPoints.size() <= minViews) {
                break;
            }
            if (errs.get(i) > limit) {
                objectPoints.get(i).release();
                imagePoints.get(i).release();
                rvecs.get(i).release();
                tvecs.get(i).release();
                objectPoints.remove(i);
                imagePoints.remove(i);
                rvecs.remove(i);
                tvecs.remove(i);
                dropped++;
            }
        }
        return dropped;
    }

    private static double perViewRms(Mat objectPoints, Mat imagePoints,
                                     Mat rvec, Mat tvec, Mat K, Mat D) {
        Mat projected = new Mat();
        try {
            Calib3d.fisheye_projectPoints(objectPoints, projected, rvec, tvec, K, D);
        } catch (CvException e) {
            projected.release();
            return Double.POSITIVE_INFINITY;
        }
        double sum = 0;
        int n = (int) imagePoints.total();
        for (int i = 0; i < n; i++) {
            double[] obs = imagePoints.get(i, 0);
            double[] pred = projected.get(i, 0);
            if (obs == null || pred == null || obs.length < 2 || pred.length < 2) {
                continue;
            }
            double dx = obs[0] - pred[0];
            double dy = obs[1] - pred[1];
            sum += dx * dx + dy * dy;
        }
        projected.release();
        return n > 0 ? Math.sqrt(sum / n) : Double.POSITIVE_INFINITY;
    }

    private static double median(List<Double> values) {
        if (values == null || values.isEmpty()) {
            return 0;
        }
        List<Double> sorted = new ArrayList<>(values);
        sorted.sort(Double::compareTo);
        int mid = sorted.size() / 2;
        if (sorted.size() % 2 == 1) {
            return sorted.get(mid);
        }
        return (sorted.get(mid - 1) + sorted.get(mid)) * 0.5;
    }

    static Mat chessboardObjectPoints(int cols, int rows, double squareMeters) {
        Point3[] pts = new Point3[cols * rows];
        int n = 0;
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                pts[n++] = new Point3(x * squareMeters, y * squareMeters, 0.0);
            }
        }
        MatOfPoint3f obj = new MatOfPoint3f(pts);
        Mat obj64 = new Mat();
        obj.convertTo(obj64, CvType.CV_64FC3);
        obj.release();
        return obj64;
    }

    private static final class PoseEstimate {
        boolean hasExtrinsics;
        double yawDeg;
        double pitchDeg;
        double rollDeg;
        double[] rvec = new double[] { 0, 0, 0 };
        double[] tvec = new double[] { 0, 0, 0 };
    }

    /**
     * Best-effort vehicle extrinsics from chessboard poses.
     * Assumes the board lies on the ground (Z out of board = up). Yaw stays
     * the configured camera role heading because board yaw in the world is unknown.
     */
    static PoseEstimate estimateVehicleExtrinsics(List<Mat> rvecs, List<Mat> tvecs,
                                                  List<Mat> imagePoints, int camIndex) {
        PoseEstimate out = new PoseEstimate();
        out.yawDeg = VideoStreamingServer.CAM_YAW_DEG[camIndex];
        out.pitchDeg = VideoStreamingServer.CAM_PITCH_DEG[camIndex];
        out.rollDeg = VideoStreamingServer.CAM_ROLL_DEG[camIndex];
        if (rvecs == null || rvecs.isEmpty() || rvecs.size() != tvecs.size()) {
            return out;
        }

        int best = 0;
        double bestSpan = -1;
        List<Double> pitches = new ArrayList<>();
        List<Double> rolls = new ArrayList<>();
        for (int i = 0; i < imagePoints.size() && i < rvecs.size(); i++) {
            double span = cornerSpan(imagePoints.get(i));
            if (span > bestSpan) {
                bestSpan = span;
                best = i;
            }
            Mat rvec = rvecs.get(i);
            if (rvec == null || rvec.empty()) {
                continue;
            }
            Mat R = new Mat();
            Calib3d.Rodrigues(rvec, R);
            if (R.rows() != 3 || R.cols() != 3) {
                R.release();
                continue;
            }
            double ox = R.get(2, 0)[0];
            double oy = R.get(2, 1)[0];
            double oz = R.get(2, 2)[0];
            double horiz = Math.hypot(ox, oy);
            double lookDownDeg = Math.toDegrees(Math.atan2(-oz, Math.max(horiz, 1e-9)));
            double pitchDeg = -lookDownDeg;
            double xx = R.get(0, 0)[0];
            double xy = R.get(0, 1)[0];
            double rollDeg = Math.toDegrees(Math.atan2(xy, xx));
            R.release();
            if (pitchDeg >= -62 && pitchDeg <= 8) {
                double flatness = Math.min(1.0, Math.abs(oz));
                if (flatness > 0.15) {
                    pitches.add(pitchDeg);
                }
                if (Math.abs(rollDeg) <= 22 && flatness > 0.15) {
                    rolls.add(rollDeg);
                }
            }
        }

        Mat rvec = rvecs.get(best);
        Mat tvec = tvecs.get(best);
        if (rvec != null && !rvec.empty() && tvec != null && !tvec.empty()) {
            out.rvec = new double[] { getCoeff(rvec, 0), getCoeff(rvec, 1), getCoeff(rvec, 2) };
            out.tvec = new double[] { getCoeff(tvec, 0), getCoeff(tvec, 1), getCoeff(tvec, 2) };
        }

        if (pitches.isEmpty()) {
            return out;
        }
        double pitchMed = median(pitches);
        double configuredPitch = VideoStreamingServer.CAM_PITCH_DEG[camIndex];
        double pitchSpread = spread(pitches);
        if (pitchSpread > 9.0 || pitches.size() < 3) {
            double w = pitches.size() < 3 ? 0.45 : 0.35;
            pitchMed = w * pitchMed + (1.0 - w) * configuredPitch;
        }
        out.pitchDeg = pitchMed;
        if (!rolls.isEmpty()) {
            out.rollDeg = rolls.size() >= 3 ? median(rolls) : median(rolls);
        }
        out.hasExtrinsics = true;
        return out;
    }

    private static double spread(List<Double> values) {
        if (values == null || values.size() < 2) {
            return 0;
        }
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (double v : values) {
            min = Math.min(min, v);
            max = Math.max(max, v);
        }
        return max - min;
    }

    private static double cornerSpan(Mat imagePoints) {
        if (imagePoints == null || imagePoints.empty()) {
            return 0;
        }
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        int n = (int) imagePoints.total();
        for (int i = 0; i < n; i++) {
            double[] p = imagePoints.get(i, 0);
            if (p == null || p.length < 2) {
                continue;
            }
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }
        if (!Double.isFinite(minX)) {
            return 0;
        }
        return Math.hypot(maxX - minX, maxY - minY);
    }

    private static double getCoeff(Mat m, int i) {
        if (m == null || m.empty()) {
            return 0;
        }
        if (m.rows() >= i + 1 && m.cols() >= 1) {
            double[] v = m.get(i, 0);
            if (v != null && v.length > 0) {
                return v[0];
            }
        }
        if (m.rows() >= 1 && m.cols() >= i + 1) {
            double[] v = m.get(0, i);
            if (v != null && v.length > 0) {
                return v[0];
            }
        }
        double[] flat = new double[(int) (m.total() * m.channels())];
        m.get(0, 0, flat);
        return i < flat.length ? flat[i] : 0;
    }

    static String toJson(CameraModel m) {
        StringBuilder sb = new StringBuilder();
        sb.append("{\n");
        field(sb, "role", m.role, true);
        field(sb, "imageWidth", m.imageWidth);
        field(sb, "imageHeight", m.imageHeight);
        field(sb, "rms", m.rms);
        field(sb, "fx", m.fx);
        field(sb, "fy", m.fy);
        field(sb, "cx", m.cx);
        field(sb, "cy", m.cy);
        field(sb, "k1", m.k1);
        field(sb, "k2", m.k2);
        field(sb, "k3", m.k3);
        field(sb, "k4", m.k4);
        field(sb, "hasIntrinsics", m.hasIntrinsics);
        field(sb, "hasExtrinsics", m.hasExtrinsics);
        field(sb, "yawDeg", m.yawDeg);
        field(sb, "pitchDeg", m.pitchDeg);
        field(sb, "rollDeg", m.rollDeg);
        array(sb, "rvec", m.rvec);
        array(sb, "tvec", m.tvec);
        sb.append("  \"_note\": \"Generated by CalibrationManager. Used at server start; not shown in UI.\"\n");
        sb.append("}\n");
        return sb.toString();
    }

    static CameraModel parseJson(String roleFallback, String json) {
        String role = strVal(json, "role", roleFallback);
        int w = (int) numVal(json, "imageWidth", 0);
        int h = (int) numVal(json, "imageHeight", 0);
        double rms = numVal(json, "rms", 0);
        double fx = numVal(json, "fx", 0);
        double fy = numVal(json, "fy", fx);
        double cx = numVal(json, "cx", w * 0.5);
        double cy = numVal(json, "cy", h * 0.5);
        double k1 = numVal(json, "k1", 0);
        double k2 = numVal(json, "k2", 0);
        double k3 = numVal(json, "k3", 0);
        double k4 = numVal(json, "k4", 0);
        boolean hasK = boolVal(json, "hasIntrinsics", fx > 0 && fy > 0);
        boolean hasE = boolVal(json, "hasExtrinsics", false);
        double yaw = numVal(json, "yawDeg", 0);
        double pitch = numVal(json, "pitchDeg", 0);
        double roll = numVal(json, "rollDeg", 0);
        double[] rvec = arrVal(json, "rvec");
        double[] tvec = arrVal(json, "tvec");
        return new CameraModel(role, w, h, rms, fx, fy, cx, cy, k1, k2, k3, k4,
                hasK, hasE, yaw, pitch, roll, rvec, tvec);
    }

    private static void field(StringBuilder sb, String k, String v, boolean quote) {
        sb.append("  \"").append(k).append("\": ");
        if (quote) {
            sb.append("\"").append(v).append("\"");
        } else {
            sb.append(v);
        }
        sb.append(",\n");
    }

    private static void field(StringBuilder sb, String k, double v) {
        sb.append("  \"").append(k).append("\": ")
                .append(String.format(Locale.US, "%.10g", v)).append(",\n");
    }

    private static void field(StringBuilder sb, String k, int v) {
        sb.append("  \"").append(k).append("\": ").append(v).append(",\n");
    }

    private static void field(StringBuilder sb, String k, boolean v) {
        sb.append("  \"").append(k).append("\": ").append(v).append(",\n");
    }

    private static void array(StringBuilder sb, String k, double[] v) {
        sb.append("  \"").append(k).append("\": [");
        if (v != null) {
            for (int i = 0; i < v.length; i++) {
                if (i > 0) sb.append(", ");
                sb.append(String.format(Locale.US, "%.10g", v[i]));
            }
        }
        sb.append("],\n");
    }

    private static String strVal(String json, String key, String fallback) {
        String q = "\"" + key + "\"";
        int i = json.indexOf(q);
        if (i < 0) return fallback;
        int colon = json.indexOf(':', i + q.length());
        int a = json.indexOf('"', colon + 1);
        int b = json.indexOf('"', a + 1);
        if (a < 0 || b < 0) return fallback;
        return json.substring(a + 1, b);
    }

    private static double numVal(String json, String key, double fallback) {
        String q = "\"" + key + "\"";
        int i = json.indexOf(q);
        if (i < 0) return fallback;
        int colon = json.indexOf(':', i + q.length());
        if (colon < 0) return fallback;
        int start = colon + 1;
        while (start < json.length() && Character.isWhitespace(json.charAt(start))) start++;
        int end = start;
        while (end < json.length()) {
            char c = json.charAt(end);
            if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
                end++;
            } else {
                break;
            }
        }
        if (end == start) return fallback;
        try {
            return Double.parseDouble(json.substring(start, end));
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static boolean boolVal(String json, String key, boolean fallback) {
        String q = "\"" + key + "\"";
        int i = json.indexOf(q);
        if (i < 0) return fallback;
        int colon = json.indexOf(':', i + q.length());
        if (colon < 0) return fallback;
        String rest = json.substring(colon + 1).trim().toLowerCase(Locale.ROOT);
        if (rest.startsWith("true")) return true;
        if (rest.startsWith("false")) return false;
        return fallback;
    }

    private static double[] arrVal(String json, String key) {
        String q = "\"" + key + "\"";
        int i = json.indexOf(q);
        if (i < 0) return new double[] { 0, 0, 0 };
        int lb = json.indexOf('[', i + q.length());
        int rb = json.indexOf(']', lb + 1);
        if (lb < 0 || rb < 0) return new double[] { 0, 0, 0 };
        String inner = json.substring(lb + 1, rb).trim();
        if (inner.isEmpty()) return new double[] { 0, 0, 0 };
        String[] parts = inner.split(",");
        double[] out = new double[parts.length];
        for (int k = 0; k < parts.length; k++) {
            try {
                out[k] = Double.parseDouble(parts[k].trim());
            } catch (NumberFormatException e) {
                out[k] = 0;
            }
        }
        return out;
    }

    private static boolean isImage(Path p) {
        if (p == null || !Files.isRegularFile(p)) {
            return false;
        }
        String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
        return n.endsWith(".jpg") || n.endsWith(".jpeg") || n.endsWith(".png")
                || n.endsWith(".bmp") || n.endsWith(".tif") || n.endsWith(".tiff");
    }

    private static void addIfUsable(List<Path> out, Path p) {
        if (VideoStreamingServer.isUsableFile(p)) {
            out.add(p);
        }
    }

    private static void releaseAll(List<Mat> mats) {
        if (mats == null) return;
        for (Mat m : mats) {
            if (m != null) m.release();
        }
        mats.clear();
    }

    static Options parseArgs(String[] args) {
        Options opt = new Options();
        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            if ("--pattern".equals(a) && i + 1 < args.length) {
                parsePattern(opt, args[++i]);
            } else if (a.startsWith("--pattern=")) {
                parsePattern(opt, a.substring("--pattern=".length()));
            } else if ("--square".equals(a) && i + 1 < args.length) {
                opt.squareMeters = parseMeters(args[++i]);
            } else if (a.startsWith("--square=")) {
                opt.squareMeters = parseMeters(a.substring("--square=".length()));
            } else if ("--min-views".equals(a) && i + 1 < args.length) {
                opt.minViews = Integer.parseInt(args[++i]);
            } else if ("--max-views".equals(a) && i + 1 < args.length) {
                opt.maxViews = Integer.parseInt(args[++i]);
            } else if ("--frame-step".equals(a) && i + 1 < args.length) {
                opt.frameStep = Integer.parseInt(args[++i]);
            } else if ("--camera".equals(a) && i + 1 < args.length) {
                opt.onlyRole = args[++i];
            } else if ("--folder".equals(a) && i + 1 < args.length) {
                opt.folder = Paths.get(args[++i]).toAbsolutePath().normalize();
            }
        }
        if (opt.minViews < 3) opt.minViews = 3;
        if (opt.maxViews < opt.minViews) opt.maxViews = opt.minViews;
        if (opt.frameStep < 1) opt.frameStep = 1;
        return opt;
    }

    private static void parsePattern(Options opt, String spec) {
        String[] p = spec.toLowerCase(Locale.ROOT).split("[x×]");
        if (p.length != 2) {
            throw new IllegalArgumentException("Pattern must be COLSxROWS, got: " + spec);
        }
        opt.patternCols = Integer.parseInt(p[0].trim());
        opt.patternRows = Integer.parseInt(p[1].trim());
    }

    private static double parseMeters(String spec) {
        String s = spec.trim().toLowerCase(Locale.ROOT);
        if (s.endsWith("mm")) {
            return Double.parseDouble(s.substring(0, s.length() - 2).trim()) / 1000.0;
        }
        if (s.endsWith("cm")) {
            return Double.parseDouble(s.substring(0, s.length() - 2).trim()) / 100.0;
        }
        return Double.parseDouble(s);
    }

    static boolean isAlignMountsArgs(String[] args) {
        if (args == null) {
            return false;
        }
        for (String a : args) {
            if ("--align-mounts".equals(a)) {
                return true;
            }
        }
        return false;
    }

    static boolean isCalibrateArgs(String[] args) {
        if (args == null) return false;
        for (String a : args) {
            if ("--calibrate".equals(a) || "-c".equals(a)) {
                return true;
            }
        }
        return false;
    }

    static Integer parsePort(String[] args) {
        if (args == null) return null;
        for (String a : args) {
            if (a.startsWith("-")) continue;
            try {
                return Integer.parseInt(a);
            } catch (NumberFormatException ignored) {
            }
        }
        return null;
    }

    static void logLoadedModels() {
        for (String role : VideoStreamingServer.CAM_ROLE) {
            CameraModel m = load(role);
            if (m == null || !m.hasIntrinsics) {
                System.out.println("  calib " + role + ": FOV fallback ("
                        + VideoStreamingServer.INPUT_FISHEYE_FOV_DEG + " deg equidistant)");
            } else if (!m.hasExtrinsics) {
                System.out.printf(Locale.US,
                        "  calib %s: intrinsics OK (rms=%.3f) but mount pose = defaults — run --align-mounts%n",
                        role, m.rms);
            } else {
                System.out.printf(Locale.US,
                        "  calib %s: K fx=%.1f fy=%.1f cx=%.1f cy=%.1f  D=[%.4f %.4f %.4f %.4f]  rms=%.3f%s%n",
                        role, m.fx, m.fy, m.cx, m.cy, m.k1, m.k2, m.k3, m.k4, m.rms,
                        m.hasExtrinsics
                                ? String.format(Locale.US, "  ext pitch=%.1f roll=%.1f",
                                m.pitchDeg, m.rollDeg)
                                : "  ext=configured");
            }
        }
    }
}
