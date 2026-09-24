import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
 
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.Executors;
 
import org.opencv.core.*;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
 
/**
 * Multi-camera panoramic projection with a shared vehicle-frame horizon.
 *
 *   fisheye pixels → 3D rays → R_camera (extrinsics) → common vehicle frame
 *        → spherical (θ, φ) → LEFT | FRONT | RIGHT | REAR canvas → feather
 *
 * Horizon is φ = 0 in the vehicle frame (pose), not an image-space shift.
 * Clips are discovered in the working folder (left / front / right / rear).
 *
 * Main calibration uses the existing clips (no checkerboard):
 *   java VideoStreamingServer --calibrate
 *   common overlap features → fisheye K,D → RANSAC → ground lines
 * Checkerboard calibration is optional:
 *   java VideoStreamingServer --calibrate-chessboard [--pattern 9x6] [--square 30mm]
 * Production loads calib/&lt;role&gt;.json when present; otherwise the
 * equidistant FOV model is used. Maps are built once, then remap() per frame.
 */
public class VideoStreamingServer {
 
    private static final int DEFAULT_PORT = 9090;
    private static final int PANEL_WIDTH  = 640;
    private static final int PANEL_HEIGHT = 400;
    /** Row of the common world horizon (fraction from the top). */
    private static final double HORIZON_FRACTION = 0.40;
    /** Horizontal field of each spherical panel (deg). >90 leaves overlap. */
    private static final double PANEL_YAW_DEG = 118.0;
    /** Vertical field around the horizon (deg). */
    private static final double PANEL_PITCH_DEG = 72.0;
    /** Equidistant fisheye input FOV used to build K when no calib file exists. */
    static final double INPUT_FISHEYE_FOV_DEG = 190.0;
/** Stop sampling past this angle from the lens axis — real fisheye glass
 *  degrades near the rim, so this keeps panels out of the bad zone. */
private static final double MAX_INCIDENCE_DEG = 78.0;
/** Optical center as a fraction of image size; 0.5/0.5 = geometric center. */
    static final double FISHEYE_CX = 0.50;
    static final double FISHEYE_CY = 0.50;
    /** Only treat near-black remap holes as invalid (keep dark asphalt). */
    private static final double INVALID_LUMA = 3.0;
    /** Minimum seam width in pixels so feeds dissolve instead of overwriting. */
    private static final int MIN_BLEND_PX = 34;
    /** Fade distance from unmapped (black) pixels. */
    private static final int EDGE_FEATHER_PX = 100;
 
    /** Vehicle yaw of each camera, Left → Front → Right → Rear. */
    static final double[] CAM_YAW_DEG   = { -90.0, 0.0, 90.0, 180.0 };
    /** Pitch: negative looks toward the ground (typical bumper / wing mount). */
    static final double[] CAM_PITCH_DEG = { -14.0, -12.0, -14.0, -21.0 };
    static final double[] CAM_ROLL_DEG  = {  0.0,  0.0,  0.0,   0.0 };
    static final String[] CAM_ROLE      = { "left", "front", "right", "rear" };
    private static final boolean[] UNSTABLE_INTRINSICS_WARNED = new boolean[4];
 
    public static void main(String[] args) throws IOException {
        try {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        } catch (UnsatisfiedLinkError e) {
            System.err.println("OpenCV native library not found: " + e.getMessage());
            return;
        }
        loadFfmpegPlugin();

        if (LineFisheyeCalibrator.isSelfTestArgs(args)) {
            int code = LineFisheyeCalibrator.selfTest();
            if (code != 0) {
                System.exit(code);
            }
            return;
        }

        if (CalibrationManager.isChessboardCalibrateArgs(args)) {
            CalibrationManager.Options opt;
            try {
                opt = CalibrationManager.parseArgs(args);
            } catch (RuntimeException e) {
                System.err.println(e.getMessage());
                return;
            }
            int code = CalibrationManager.run(opt);
            Path calibFolder = opt.folder != null
                    ? opt.folder : Paths.get(".").toAbsolutePath().normalize();
            int align = alignMountPoseFromSeams(calibFolder);
            if (code != 0 && align != 0) {
                System.exit(code);
            }
            return;
        }

        if (CalibrationManager.isCalibrateArgs(args)) {
            CalibrationManager.Options opt;
            try {
                opt = CalibrationManager.parseArgs(args);
            } catch (RuntimeException e) {
                System.err.println(e.getMessage());
                return;
            }
            int code = LineFisheyeCalibrator.run(opt.folder);
            if (code != 0) {
                System.exit(code);
            }
            return;
        }

        if (isStitchPreviewArgs(args)) {
            Path folder = Paths.get(".").toAbsolutePath().normalize();
            int code = exportStitchPreview(folder);
            if (code != 0) {
                System.exit(code);
            }
            return;
        }

        if (CalibrationManager.isAlignMountsArgs(args)) {
            Path folder = Paths.get(".").toAbsolutePath().normalize();
            int code = alignMountPoseFromSeams(folder);
            if (code != 0) {
                System.exit(code);
            }
            return;
        }

        int port = DEFAULT_PORT;
        Integer parsedPort = CalibrationManager.parsePort(args);
        if (parsedPort != null) {
            port = parsedPort;
        } else if (args.length >= 1 && !args[0].startsWith("-")) {
            System.err.println("First argument must be a port number or --calibrate, got: " + args[0]);
            return;
        }

        Path folder = Paths.get(".").toAbsolutePath().normalize();
        Path[] videos = discoverClips(folder);
        if (videos == null) {
            return;
        }

        HttpServer server = HttpServer.create(new InetSocketAddress(port), 0);
        server.createContext("/stitch", new StitchHandler(videos));
        server.createContext("/play",   new PlayerPageHandler());
        server.setExecutor(Executors.newFixedThreadPool(4));
        server.start();

        System.out.println("Server started  ->  http://localhost:" + port + "/play");
        CalibrationManager.logLoadedModels();
        for (int i = 0; i < CAM_ROLE.length; i++) {
            System.out.println("  " + CAM_ROLE[i] + " = " + videos[i]
                    + "  yaw=" + CAM_YAW_DEG[i]
                    + " pitch=" + CAM_PITCH_DEG[i]
                    + " roll=" + CAM_ROLL_DEG[i]);
        }
    }
 
    /**
     * Finds left/front/right/rear clips in {@code folder}. Prefers {@code name_1.mp4}
     * then {@code name.mp4}/{@code .mov}, then any file whose name contains the role.
     */
    static boolean isStitchPreviewArgs(String[] args) {
        if (args == null) {
            return false;
        }
        for (String a : args) {
            if ("--stitch-preview".equals(a)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Renders one offline panorama JPEG from a still per camera (for tuning calib).
     * Looks for calib/&lt;role&gt;/preview.jpg, then dummy_00.jpg, then any image in the folder.
     */
    static int exportStitchPreview(Path folder) {
        Mat[] frames = new Mat[CAM_ROLE.length];
        for (int i = 0; i < CAM_ROLE.length; i++) {
            String source = loadPreviewFrame(folder, CAM_ROLE[i], frames, i);
            if (source == null) {
                for (Mat m : frames) {
                    if (m != null) {
                        m.release();
                    }
                }
                return 2;
            }
            System.out.println("Preview source " + CAM_ROLE[i] + ": " + source);
        }
        Mat panorama = renderPanorama(frames, null, null);
        for (Mat m : frames) {
            if (m != null) {
                m.release();
            }
        }
        if (panorama == null) {
            return 1;
        }
        Path out = folder.resolve("stitch_preview.jpg");
        boolean ok = Imgcodecs.imwrite(out.toAbsolutePath().toString(), panorama);
        panorama.release();
        if (!ok) {
            System.err.println("Could not write " + out);
            return 1;
        }
        System.out.println("Wrote " + out.toAbsolutePath());
        return 0;
    }

    /**
     * Align camera mounts so the same scene point lands on the same panorama
     * direction. Uses ORB matches between adjacent cameras (the overlap of a
     * 360 rig). Checkerboard intrinsics, when present, supply the fisheye model
     * that turns pixels into rays. Front-camera yaw stays fixed as the reference.
     */
    static int alignMountPoseFromSeams(Path folder) {
        Path[] clips = discoverClips(folder);
        if (clips == null) {
            System.err.println("Mount align skipped: left/front/right/rear clips not found.");
            return 2;
        }
        Mat[] frames = new Mat[CAM_ROLE.length];
        for (int i = 0; i < CAM_ROLE.length; i++) {
            if (!readFirstFrame(clips[i], frames, i)) {
                System.err.println("Could not read a frame from " + clips[i]);
                releaseFrames(frames);
                return 2;
            }
        }

        double[] yaw = new double[CAM_ROLE.length];
        double[] pitch = new double[CAM_ROLE.length];
        double[] roll = new double[CAM_ROLE.length];
        for (int i = 0; i < CAM_ROLE.length; i++) {
            CalibrationManager.CameraModel model = CalibrationManager.load(CAM_ROLE[i]);
            double[] pose = CalibrationManager.mountDegreesForStitch(
                    i, model, Double.NaN, Double.NaN);
            yaw[i] = pose[0];
            pitch[i] = pose[1];
            roll[i] = pose[2];
        }

        List<RayMatch> matches = collectAdjacentRayMatches(frames);
        System.out.println("Cross-camera feature matches: " + matches.size());
        if (matches.size() < 12) {
            System.err.println("Not enough overlap matches to solve mount pose.");
            System.err.println("Place the rig so neighboring cameras see the same street/vehicles,");
            System.err.println("or capture a checkerboard large enough to appear in two cameras at once.");
            releaseFrames(frames);
            return 2;
        }

        double before = featurePoseCost(matches, yaw, pitch, roll);
        System.out.printf(Locale.US, "Feature ray error before align: %.3f deg%n", before);

        double[] yawStep = { 4.0, 1.0 };
        double[] pitchStep = { 4.0, 1.0 };
        double[] rollStep = { 2.0, 0.5 };
        double[] yawRange = { 16.0, 4.0 };
        double[] pitchRange = { 16.0, 4.0 };
        double[] rollRange = { 6.0, 2.0 };
        for (int pass = 0; pass < 2; pass++) {
            for (int cam = 0; cam < CAM_ROLE.length; cam++) {
                double baseYaw = yaw[cam];
                double basePitch = pitch[cam];
                double baseRoll = roll[cam];
                double bestYaw = baseYaw;
                double bestPitch = basePitch;
                double bestRoll = baseRoll;
                double best = featurePoseCost(matches, yaw, pitch, roll);
                boolean lockYaw = cam == 1;
                for (double dy = lockYaw ? 0 : -yawRange[pass]; dy <= yawRange[pass] + 1e-6; dy += yawStep[pass]) {
                    for (double dp = -pitchRange[pass]; dp <= pitchRange[pass] + 1e-6; dp += pitchStep[pass]) {
                        for (double dr = -rollRange[pass]; dr <= rollRange[pass] + 1e-6; dr += rollStep[pass]) {
                            yaw[cam] = baseYaw + dy;
                            pitch[cam] = basePitch + dp;
                            roll[cam] = baseRoll + dr;
                            double score = featurePoseCost(matches, yaw, pitch, roll);
                            if (score < best) {
                                best = score;
                                bestYaw = yaw[cam];
                                bestPitch = pitch[cam];
                                bestRoll = roll[cam];
                            }
                        }
                    }
                }
                yaw[cam] = bestYaw;
                pitch[cam] = bestPitch;
                roll[cam] = bestRoll;
                System.out.printf(Locale.US,
                        "  %s pass %d: yaw=%.2f pitch=%.2f roll=%.2f  err=%.3f deg%n",
                        CAM_ROLE[cam], pass + 1, yaw[cam], pitch[cam], roll[cam], best);
            }
        }

        double after = featurePoseCost(matches, yaw, pitch, roll);
        System.out.printf(Locale.US, "Feature ray error after align: %.3f deg%n", after);
        if (after > before - 0.05) {
            System.out.println("Align did not improve the fit; keeping the previous mount pose.");
            releaseFrames(frames);
            return 0;
        }

        for (int i = 0; i < CAM_ROLE.length; i++) {
            if (!CalibrationManager.saveMountPose(CAM_ROLE[i], yaw[i], pitch[i], roll[i])) {
                releaseFrames(frames);
                return 1;
            }
        }
        releaseFrames(frames);
        System.out.println("Mount pose saved to calib/*.json (hasExtrinsics=true).");
        System.out.println("Next: java VideoStreamingServer --stitch-preview");
        return 0;
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

    private static List<RayMatch> collectAdjacentRayMatches(Mat[] frames) {
        List<RayMatch> out = new ArrayList<>();
        ORB orb = ORB.create(2000);
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        Mat[] gray = new Mat[frames.length];
        MatOfKeyPoint[] keys = new MatOfKeyPoint[frames.length];
        Mat[] desc = new Mat[frames.length];
        for (int i = 0; i < frames.length; i++) {
            gray[i] = new Mat();
            Imgproc.cvtColor(frames[i], gray[i], Imgproc.COLOR_BGR2GRAY);
            keys[i] = new MatOfKeyPoint();
            desc[i] = new Mat();
            orb.detectAndCompute(gray[i], new Mat(), keys[i], desc[i]);
        }
        for (int i = 0; i < frames.length - 1; i++) {
            if (desc[i].empty() || desc[i + 1].empty()) {
                continue;
            }
            List<MatOfDMatch> knn = new ArrayList<>();
            matcher.knnMatch(desc[i], desc[i + 1], knn, 2);
            KeyPoint[] ka = keys[i].toArray();
            KeyPoint[] kb = keys[i + 1].toArray();
            double[][] nominal = new double[2][];
            nominal[0] = cameraToVehicle(CAM_YAW_DEG[i], CAM_PITCH_DEG[i], CAM_ROLL_DEG[i]);
            nominal[1] = cameraToVehicle(CAM_YAW_DEG[i + 1], CAM_PITCH_DEG[i + 1], CAM_ROLL_DEG[i + 1]);
            for (MatOfDMatch pair : knn) {
                DMatch[] dm = pair.toArray();
                if (dm.length < 2 || dm[0].distance > 0.75 * dm[1].distance) {
                    continue;
                }
                KeyPoint pa = ka[dm[0].queryIdx];
                KeyPoint pb = kb[dm[0].trainIdx];
                double[] rayA = pixelToCameraRay(frames[i], i, pa.pt.x, pa.pt.y);
                double[] rayB = pixelToCameraRay(frames[i + 1], i + 1, pb.pt.x, pb.pt.y);
                if (rayA == null || rayB == null) {
                    continue;
                }
                double[] va = mulMatVec(nominal[0], rayA);
                double[] vb = mulMatVec(nominal[1], rayB);
                if (angleDeg(va, vb) > 40.0) {
                    continue;
                }
                out.add(new RayMatch(i, i + 1, rayA, rayB));
            }
        }
        for (int i = 0; i < frames.length; i++) {
            gray[i].release();
            keys[i].release();
            desc[i].release();
        }
        return out;
    }

    private static double[] pixelToCameraRay(Mat frame, int cam, double u, double v) {
        CalibrationManager.CameraModel model = CalibrationManager.load(CAM_ROLE[cam]);
        double f = fisheyeFocal(frame.cols(), frame.rows(), INPUT_FISHEYE_FOV_DEG);
        double[] K = (model != null && model.hasIntrinsics)
                ? model.scaledK(frame.cols(), frame.rows())
                : new double[] { f, f, frame.cols() * FISHEYE_CX, frame.rows() * FISHEYE_CY };
        double k1 = model != null && model.hasIntrinsics ? model.k1 : 0;
        double k2 = model != null && model.hasIntrinsics ? model.k2 : 0;
        double k3 = model != null && model.hasIntrinsics ? model.k3 : 0;
        double k4 = model != null && model.hasIntrinsics ? model.k4 : 0;
        return CalibrationManager.unprojectFisheye(u, v, K, k1, k2, k3, k4);
    }

    private static double featurePoseCost(List<RayMatch> matches,
                                          double[] yaw, double[] pitch, double[] roll) {
        double[][] r = new double[yaw.length][];
        for (int i = 0; i < yaw.length; i++) {
            r[i] = cameraToVehicle(yaw[i], pitch[i], roll[i]);
        }
        double sum = 0;
        int n = 0;
        for (RayMatch m : matches) {
            double ang = angleDeg(mulMatVec(r[m.a], m.rayA), mulMatVec(r[m.b], m.rayB));
            sum += Math.min(ang, 15.0);
            n++;
        }
        if (n == 0) {
            return 1e6;
        }
        double prior = 0;
        for (int i = 0; i < yaw.length; i++) {
            prior += Math.abs(yaw[i] - CAM_YAW_DEG[i]) * 0.02;
            prior += Math.abs(pitch[i] - CAM_PITCH_DEG[i]) * 0.02;
            prior += Math.abs(roll[i] - CAM_ROLL_DEG[i]) * 0.02;
        }
        return sum / n + prior;
    }

    private static double[] mulMatVec(double[] r, double[] v) {
        return new double[] {
            r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
            r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2]
        };
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

    private static void releaseFrames(Mat[] frames) {
        if (frames == null) {
            return;
        }
        for (Mat frame : frames) {
            if (frame != null) {
                frame.release();
            }
        }
    }

    private static boolean readFirstFrame(Path clip, Mat[] frames, int index) {
        Path decodable = ensureDecodable(clip);
        VideoCapture cap = openVideo(decodable);
        if (cap == null || !cap.isOpened()) {
            return false;
        }
        Mat frame = new Mat();
        boolean ok = readBgr(cap, frame) && !frame.empty();
        cap.release();
        if (!ok) {
            frame.release();
            return false;
        }
        frames[index] = frame;
        return true;
    }

    private static String loadPreviewFrame(Path folder, String role, Mat[] frames, int index) {
        Path clip = findClip(folder, role);
        if (clip != null && readFirstFrame(ensureDecodable(clip), frames, index)) {
            return clip.getFileName().toString() + " (first frame)";
        }
        Path still = findCalibStill(folder, role);
        if (still == null) {
            System.err.println("No preview for " + role
                    + " (need " + role + "_1.mp4 or calib/" + role + "/ still)");
            return null;
        }
        Mat bgr = Imgcodecs.imread(still.toAbsolutePath().toString());
        if (bgr == null || bgr.empty()) {
            System.err.println("Could not read " + still);
            return null;
        }
        frames[index] = bgr;
        return still.getFileName().toString();
    }

    static Mat renderPanorama(Mat[] frames, double[] pitchDeg, double[] rollDeg) {
        if (frames == null || frames.length != CAM_ROLE.length) {
            return null;
        }
        Mat[] ready = new Mat[CAM_ROLE.length];
        for (int i = 0; i < CAM_ROLE.length; i++) {
            if (frames[i] == null || frames[i].empty()) {
                for (Mat m : ready) {
                    if (m != null) {
                        m.release();
                    }
                }
                return null;
            }
            double p = pitchDeg != null ? pitchDeg[i] : Double.NaN;
            double r = rollDeg != null ? rollDeg[i] : Double.NaN;
            SphericalPanel panel = new SphericalPanel(i, Double.NaN, p, r);
            ready[i] = new Mat();
            panel.project(frames[i], ready[i]);
        }
        int overlap = panelOverlapPx();
        Mat panorama = featherStitch(ready, overlap);
        for (Mat m : ready) {
            m.release();
        }
        return panorama;
    }


    private static Path findCalibStill(Path folder, String role) {
        Path dir = folder.resolve(CalibrationManager.CALIB_DIR).resolve(role);
        Path[] preferred = {
            dir.resolve("preview.jpg"),
            dir.resolve("dummy_00.jpg"),
            dir.resolve("frame_00.jpg")
        };
        for (Path p : preferred) {
            if (isUsableFile(p)) {
                return p;
            }
        }
        if (!Files.isDirectory(dir)) {
            return null;
        }
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(dir)) {
            for (Path p : stream) {
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                if (n.endsWith(".jpg") || n.endsWith(".jpeg") || n.endsWith(".png")) {
                    return p;
                }
            }
        } catch (IOException ignored) {
        }
        return null;
    }

    static Path[] discoverClips(Path folder) {
        Path[] found = new Path[4];
        for (int i = 0; i < CAM_ROLE.length; i++) {
            found[i] = findClip(folder, CAM_ROLE[i]);
            if (found[i] == null) {
                System.err.println("No " + CAM_ROLE[i]
                        + " clip in " + folder
                        + " (expected e.g. " + CAM_ROLE[i] + "_1.mp4 or "
                        + CAM_ROLE[i] + ".mp4)");
                return null;
            }
            found[i] = ensureDecodable(found[i]);
        }
        return found;
    }
 
    static Path findClip(Path folder, String role) {
        String r = role.toLowerCase(Locale.ROOT);
        String[] preferred = {
            r + "_1.mp4", r + ".mp4", r + "_1.mov", r + ".mov",
            r + "_1.MP4", r + ".MP4"
        };
        for (String name : preferred) {
            Path p = folder.resolve(name);
            if (isUsableFile(p)) {
                return p;
            }
        }
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(folder)) {
            Path fallback = null;
            for (Path p : stream) {
                if (!isUsableFile(p)) continue;
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                if (!(n.endsWith(".mp4") || n.endsWith(".mov") || n.endsWith(".avi"))) {
                    continue;
                }
                if (n.startsWith(r + "_") || n.startsWith(r + ".") || n.equals(r + ".mp4")) {
                    return p;
                }
                if (fallback == null && n.contains(r)) {
                    fallback = p;
                }
            }
            return fallback;
        } catch (IOException e) {
            return null;
        }
    }
 
    private static class StitchHandler implements HttpHandler {
        private final Path[] videoFiles;
        StitchHandler(Path[] f) { this.videoFiles = f; }
 
        @Override public void handle(HttpExchange ex) throws IOException {
            if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
                ex.sendResponseHeaders(405, -1);
                return;
            }
 
            VideoCapture[] caps = new VideoCapture[videoFiles.length];
            for (int i = 0; i < videoFiles.length; i++) {
                caps[i] = openVideo(videoFiles[i]);
                if (caps[i] == null || !caps[i].isOpened()) {
                    System.err.println("Could not open video: " + videoFiles[i]);
                    ex.sendResponseHeaders(500, -1);
                    return;
                }
            }
 
            SphericalPanel[] panels = new SphericalPanel[videoFiles.length];
            for (int i = 0; i < panels.length; i++) {
                panels[i] = new SphericalPanel(i);
            }
 
            int overlap = panelOverlapPx();
            ex.getResponseHeaders().set("Content-Type",
                    "multipart/x-mixed-replace; boundary=frame");
            ex.sendResponseHeaders(200, 0);
 
            try (OutputStream out = ex.getResponseBody()) {
                Mat[] frames = new Mat[videoFiles.length];
                Mat[] ready  = new Mat[videoFiles.length];
                for (int i = 0; i < videoFiles.length; i++) {
                    frames[i] = new Mat();
                    ready[i]  = new Mat();
                }
 
                while (true) {
                    boolean allReady = true;
                    for (int i = 0; i < caps.length; i++) {
                        if (!readOrLoop(caps, i, videoFiles[i], frames[i])) {
                            allReady = false;
                            continue;
                        }
                        panels[i].project(frames[i], ready[i]);
                        if (ready[i].empty()
                                || ready[i].cols() != PANEL_WIDTH
                                || ready[i].rows() != PANEL_HEIGHT) {
                            allReady = false;
                        }
                    }
                    if (!allReady) {
                        continue;
                    }
                    Mat panorama = featherStitch(ready, overlap);
                    writeFrame(out, encodeJpeg(panorama));
                    panorama.release();
                }
            } finally {
                for (VideoCapture c : caps) {
                    if (c != null) c.release();
                }
            }
        }
    }
 
    private static class PlayerPageHandler implements HttpHandler {
        @Override public void handle(HttpExchange ex) throws IOException {
            String html = "<!DOCTYPE html><html lang='en'><head>"
                + "<meta charset='UTF-8'>"
                + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                + "<title>Common-horizon panorama</title>"
                + "<style>"
                + "*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }"
                + "html, body { height: 100%; background: #0a0a0f; color: #e0e0e0;"
                + "  font-family: 'Segoe UI', sans-serif; overflow: hidden; }"
                + ".container { display: flex; flex-direction: column;"
                + "  align-items: center; justify-content: center;"
                + "  height: 100vh; padding: 16px; gap: 12px; }"
                + "h1 { font-size: 1.25rem; font-weight: 300; letter-spacing: 1px;"
                + "  color: #7ec8e3; text-align: center; flex-shrink: 0; }"
                + ".pano-wrap { width: 100%; flex: 1; min-height: 0;"
                + "  border: 1px solid #2a2a3a; border-radius: 8px; overflow: hidden;"
                + "  display: flex; align-items: center; justify-content: center; }"
                + ".pano-wrap img { width: 100%; height: 100%; object-fit: contain; display: block; }"
                + "</style></head><body>"
                + "<div class='container'>"
                + "  <h1>common vehicle horizon</h1>"
                + "  <div class='pano-wrap'>"
                + "    <img src='/stitch' alt='stitched panorama'>"
                + "  </div>"
                + "</div></body></html>";
            byte[] bytes = html.getBytes("UTF-8");
            ex.getResponseHeaders().set("Content-Type", "text/html; charset=UTF-8");
            ex.sendResponseHeaders(200, bytes.length);
            try (OutputStream os = ex.getResponseBody()) { os.write(bytes); }
        }
    }
 
    /**
     * One surround camera: spherical dest pixels are inverse-projected through
     * the vehicle frame and the fisheye model into the raw frame.
     */
    static final class SphericalPanel {
        private final int index;
        private final double[] R; // camera-to-vehicle, row-major
        private final CalibrationManager.CameraModel calib;
        private Mat map1;
        private Mat map2;
        private int cachedSrcW = -1;
        private int cachedSrcH = -1;
 
        private final double mountYawDeg;
        private final double mountPitchDeg;
        private final double mountRollDeg;

        SphericalPanel(int index) {
            this(index, Double.NaN, Double.NaN, Double.NaN);
        }

        /**
         * Overrides are absolute mount degrees. NaN keeps the calibrated or default
         * mount. The panorama window stays on the nominal camera yaw so a yaw
         * correction slides image content instead of moving the panel with it.
         */
        SphericalPanel(int index, double yawOverride, double pitchOverride, double rollOverride) {
            this.index = index;
            this.calib = CalibrationManager.load(CAM_ROLE[index]);
            double[] pose = CalibrationManager.mountDegreesForStitch(
                    index, this.calib, pitchOverride, rollOverride);
            if (!Double.isNaN(yawOverride)) {
                pose[0] = yawOverride;
            }
            this.mountYawDeg = pose[0];
            this.mountPitchDeg = pose[1];
            this.mountRollDeg = pose[2];
            this.R = cameraToVehicle(mountYawDeg, mountPitchDeg, mountRollDeg);
        }
 
        void project(Mat src, Mat dst) {
            if (src == null || src.empty()) {
                return;
            }
            ensureMaps(src.cols(), src.rows());
            Imgproc.remap(src, dst, map1, map2, Imgproc.INTER_LINEAR,
                    Core.BORDER_CONSTANT, new Scalar(0, 0, 0));
        }

        private void ensureMaps(int srcW, int srcH) {
            if (map1 != null && srcW == cachedSrcW && srcH == cachedSrcH) {
                return;
            }
            boolean useCalib = CalibrationManager.stableIntrinsics(calib);
            if (calib != null && calib.hasIntrinsics && !useCalib
                    && !UNSTABLE_INTRINSICS_WARNED[index]) {
                UNSTABLE_INTRINSICS_WARNED[index] = true;
                System.out.println("  calib " + CAM_ROLE[index]
                        + ": distortion would warp the image, using equidistant lens");
            }
            double f = fisheyeFocal(srcW, srcH, INPUT_FISHEYE_FOV_DEG);
            double[] K = useCalib
                    ? calib.scaledK(srcW, srcH)
                    : new double[] { f, f, srcW * FISHEYE_CX, srcH * FISHEYE_CY };
            double k1 = useCalib ? calib.k1 : 0.0;
            double k2 = useCalib ? calib.k2 : 0.0;
            double k3 = useCalib ? calib.k3 : 0.0;
            double k4 = useCalib ? calib.k4 : 0.0;
            double cx = K[2];
            double cy = K[3];
            double maxInc = Math.toRadians(MAX_INCIDENCE_DEG);
            double yaw0 = Math.toRadians(CAM_YAW_DEG[index]);
            double yawSpan = Math.toRadians(PANEL_YAW_DEG);
            double pitchSpan = Math.toRadians(PANEL_PITCH_DEG);
            double horizonY = HORIZON_FRACTION * PANEL_HEIGHT;
 
            Mat mapX = new Mat(PANEL_HEIGHT, PANEL_WIDTH, CvType.CV_32FC1);
            Mat mapY = new Mat(PANEL_HEIGHT, PANEL_WIDTH, CvType.CV_32FC1);
            float[] rowX = new float[PANEL_WIDTH];
            float[] rowY = new float[PANEL_WIDTH];
 
            for (int v = 0; v < PANEL_HEIGHT; v++) {
                // φ = 0 on the shared horizon row; positive φ is sky (up).
                double phi = (horizonY - v) / PANEL_HEIGHT * pitchSpan;
                double cphi = Math.cos(phi);
                double sphi = Math.sin(phi);
                for (int u = 0; u < PANEL_WIDTH; u++) {
                    double theta = yaw0 + ((u + 0.5) / PANEL_WIDTH - 0.5) * yawSpan;
                    // Vehicle frame: X forward, Y right, Z up.
                    double xv = cphi * Math.cos(theta);
                    double yv = cphi * Math.sin(theta);
                    double zv = sphi;
                    // ray_camera = R^T * ray_vehicle  (OpenCV: X right, Y down, Z forward)
                    double xc = R[0] * xv + R[3] * yv + R[6] * zv;
                    double yc = R[1] * xv + R[4] * yv + R[7] * zv;
                    double zc = R[2] * xv + R[5] * yv + R[8] * zv;
                    if (zc <= 1e-4) {
                        rowX[u] = -1f;
                        rowY[u] = -1f;
                        continue;
                    }
                                        double inc = Math.atan2(Math.hypot(xc, yc), zc);
                    if (inc > maxInc) {
                        rowX[u] = -1f;
                        rowY[u] = -1f;
                        continue;
                    }
                    float su;
                    float sv;
                    if (useCalib) {
                        float[] uv = CalibrationManager.projectFisheye(
                                xc, yc, zc, K, k1, k2, k3, k4);
                        if (uv == null) {
                            rowX[u] = -1f;
                            rowY[u] = -1f;
                            continue;
                        }
                        su = uv[0];
                        sv = uv[1];
                    } else {
                        double r = fisheyeRadius(inc, f);
                        double az = Math.atan2(yc, xc);
                        su = (float) (cx + r * Math.cos(az));
                        sv = (float) (cy + r * Math.sin(az));
                    }
                    if (su < 1 || sv < 1 || su >= srcW - 1 || sv >= srcH - 1) {
                        rowX[u] = -1f;
                        rowY[u] = -1f;
                    } else {
                        rowX[u] = su;
                        rowY[u] = sv;
                    }
                }
                mapX.put(v, 0, rowX);
                mapY.put(v, 0, rowY);
            }
 
            if (map1 == null) map1 = new Mat();
            if (map2 == null) map2 = new Mat();
            Imgproc.convertMaps(mapX, mapY, map1, map2, CvType.CV_16SC2, false);
            mapX.release();
            mapY.release();
            cachedSrcW = srcW;
            cachedSrcH = srcH;
        }
    }

    /** One spherical panel, used by the line-calibration projection stage. */
    static Mat projectPanel(Mat src, int index) {
        SphericalPanel panel = new SphericalPanel(index);
        Mat dst = new Mat();
        panel.project(src, dst);
        return dst;
    }
 
    /**
     * R maps OpenCV camera rays to the vehicle frame:
     *   ray_vehicle = R * ray_camera
     * Front-looking camera (yaw=pitch=roll=0): Z_cam → X_veh, X_cam → Y_veh,
     * -Y_cam → Z_veh.
     */
    static double[] cameraToVehicle(double yawDeg, double pitchDeg, double rollDeg) {
        double[] rcv = {
            0, 0, 1,
            1, 0, 0,
            0,-1, 0
        };
        double[] rx = rotX(Math.toRadians(rollDeg));
        // Negative pitchDeg looks toward the ground (optical axis below the horizon).
        double[] ry = rotY(Math.toRadians(-pitchDeg));
        double[] rz = rotZ(Math.toRadians(yawDeg));
        return mul3(rz, mul3(ry, mul3(rx, rcv)));
    }
 
    static double[] rotX(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[] {
            1, 0, 0,
            0, c,-s,
            0, s, c
        };
    }
 
    static double[] rotY(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[] {
             c, 0, s,
             0, 1, 0,
            -s, 0, c
        };
    }
 
    static double[] rotZ(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new double[] {
            c,-s, 0,
            s, c, 0,
            0, 0, 1
        };
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
 
    static double fisheyeFocal(int width, int height, double fovDeg) {
        double half = Math.toRadians(fovDeg) / 2.0;
        return (Math.min(width, height) / 2.0) / half;
    }
 
    static double fisheyeRadius(double incidence, double focal) {
    return focal * incidence; // pure equidistant, no distortion terms yet
}
 
    static int panelOverlapPx() {
        // Adjacent cameras are 90° apart. A wider panel FOV must be blended
        // across that extra angle, or the seam joins two different directions.
        double extraDeg = PANEL_YAW_DEG - 90.0;
        int geometric = (int) Math.round(PANEL_WIDTH * extraDeg / PANEL_YAW_DEG);
        return Math.max(MIN_BLEND_PX, Math.min(PANEL_WIDTH / 2, geometric));
    }
 
    static Mat featherStitch(Mat[] frames, int overlap) {
        int N = frames.length;
        int H = PANEL_HEIGHT;
        int W = PANEL_WIDTH;
        int panoW = W + (N - 1) * (W - overlap);
        double[] gain = sequentialGains(frames, overlap);
 
                Mat accumColor  = Mat.zeros(H, panoW, CvType.CV_32FC3);
        Mat accumWeight = Mat.zeros(H, panoW, CvType.CV_32FC1);
        // Tracks the single best (highest-confidence) source pixel seen so
        // far at each column, so low-confidence blend regions can fall back
        // to real camera data instead of being blacked out.
        Mat maxWeight = Mat.zeros(H, panoW, CvType.CV_32FC1);
        Mat maxColor  = Mat.zeros(H, panoW, CvType.CV_32FC3);
 
        for (int i = 0; i < N; i++) {
            int xStart = i * (W - overlap);
            // Incoming panel ramps in slowly so it does not stamp over the
            // previous feed (the left→front overwrite). Outgoing stays visible
            // longer into the seam.
            Mat weight = buildFeatherMask(H, W, overlap, i > 0, i < N - 1, 0.55, 0.55);
            Mat edgeW = edgeDistanceWeight(frames[i], EDGE_FEATHER_PX);
            Core.multiply(weight, edgeW, weight);
            edgeW.release();
 
            Mat frameF = new Mat();
            frames[i].convertTo(frameF, CvType.CV_32FC3);
            if (Math.abs(gain[i] - 1.0) > 0.01) {
                Core.multiply(frameF, new Scalar(gain[i], gain[i], gain[i]), frameF);
            }
            Mat weight3 = new Mat();
            List<Mat> ch = new ArrayList<>();
            ch.add(weight); ch.add(weight); ch.add(weight);
            Core.merge(ch, weight3);
            Mat wFrame = new Mat();
            Core.multiply(frameF, weight3, wFrame);
 
            int xEnd = Math.min(xStart + W, panoW);
            int wActual = xEnd - xStart;
            Mat colorRoi  = accumColor.submat(0, H, xStart, xEnd);
            Mat weightRoi = accumWeight.submat(0, H, xStart, xEnd);
                        Core.add(colorRoi,  wFrame.colRange(0, wActual), colorRoi);
            Core.add(weightRoi, weight.colRange(0, wActual), weightRoi);
 
            Mat maxColorRoi  = maxColor.submat(0, H, xStart, xEnd);
            Mat maxWeightRoi = maxWeight.submat(0, H, xStart, xEnd);
            Mat greaterMask = new Mat();
            Core.compare(weight.colRange(0, wActual), maxWeightRoi, greaterMask, Core.CMP_GT);
            frameF.colRange(0, wActual).copyTo(maxColorRoi, greaterMask);
            weight.colRange(0, wActual).copyTo(maxWeightRoi, greaterMask);
            greaterMask.release();
            maxColorRoi.release();
            maxWeightRoi.release();
 
            colorRoi.release();
            weightRoi.release();
            frameF.release();
            weight.release();
            weight3.release();
            wFrame.release();
        }
 
                Mat safeW = new Mat();
        Core.max(accumWeight, new Scalar(1e-6), safeW);
        Mat safeW3 = new Mat();
        List<Mat> wch = new ArrayList<>();
        wch.add(safeW); wch.add(safeW); wch.add(safeW);
        Core.merge(wch, safeW3);
        Mat blended = new Mat();
        Core.divide(accumColor, safeW3, blended);
 
        // Anywhere total confidence is too low, the divide above blows up
        // small residual color into a bright artifact — force those pixels
        // to black instead of trusting the division.
        Mat lowWeightMask = new Mat();
        Core.compare(accumWeight, new Scalar(0.02), lowWeightMask, Core.CMP_LT);
        maxColor.copyTo(blended, lowWeightMask);
        lowWeightMask.release();
 
        Mat result = new Mat();
        blended.convertTo(result, CvType.CV_8UC3);
 
                accumColor.release();
        accumWeight.release();
        maxWeight.release();
        maxColor.release();
        safeW.release();
        safeW3.release();
        blended.release();
        return result;
    }
 
 
    /**
     * Raised-cosine seam. {@code inExp} &gt; 1 makes the new panel fade in slowly;
     * {@code outExp} &lt; 1 keeps the old panel visible further into the overlap.
     */
    static Mat buildFeatherMask(int H, int W, int overlap,
                                boolean fadeLeft, boolean fadeRight,
                                double inExp, double outExp) {
        Mat mask = new Mat(H, W, CvType.CV_32FC1, new Scalar(1.0));
        if (overlap <= 0) {
            return mask;
        }
        int feather = Math.min(36, overlap);
        int center0 = Math.max(0, (overlap - feather) / 2);
        for (int x = 0; x < overlap; x++) {
            if (fadeLeft) {
                float alpha = seamWeight(x, center0, feather, true, inExp);
                Mat colL = mask.col(x);
                colL.setTo(new Scalar(alpha));
                colL.release();
            }
            if (fadeRight) {
                int offset = overlap - 1 - x;
                float alpha = seamWeight(offset, center0, feather, false, outExp);
                Mat colR = mask.col(W - 1 - x);
                colR.setTo(new Scalar(alpha));
                colR.release();
            }
        }
        return mask;
    }

    /** One camera owns each side of a wide overlap; only the center band mixes. */
    private static float seamWeight(int offset, int center0, int feather,
                                    boolean fadeIn, double exp) {
        double w;
        if (offset < center0) {
            w = fadeIn ? 0 : 1;
        } else if (offset >= center0 + feather) {
            w = fadeIn ? 1 : 0;
        } else {
            double t = (offset - center0) / (double) feather;
            double cosine = 0.5 - 0.5 * Math.cos(Math.PI * t);
            w = fadeIn ? cosine : 1.0 - cosine;
        }
        if (exp > 0 && exp != 1.0 && w > 0 && w < 1) {
            w = fadeIn ? Math.pow(w, exp) : 1.0 - Math.pow(1.0 - w, exp);
        }
        return (float) w;
    }
 
        static Mat edgeDistanceWeight(Mat bgr, int radius) {
        Mat gray = new Mat();
        Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
        Mat mask = new Mat();
        Imgproc.threshold(gray, mask, INVALID_LUMA, 255, Imgproc.THRESH_BINARY);
        // Real remap gaps are large solid black regions; small dark specks
        // (tires, shadows, window tint) are valid content, not gaps.
        // Close them up so they don't get treated as missing data.
        Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(35, 35));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);
        closeKernel.release();
        Mat dist = new Mat();
        Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 3);
        Mat weight = new Mat();
        double scale = radius < 1 ? 1.0 : 1.0 / radius;
        dist.convertTo(weight, CvType.CV_32FC1, scale);
        Core.min(weight, new Scalar(1.0), weight);
        gray.release();
        mask.release();
        dist.release();
        return weight;
    }
 
    static double[] sequentialGains(Mat[] frames, int overlap) {
        double[] gain = new double[frames.length];
        java.util.Arrays.fill(gain, 1.0);
        for (int i = 1; i < frames.length; i++) {
            double left = overlapMean(frames[i - 1], true, overlap);
            double right = overlapMean(frames[i], false, overlap);
            if (left < 8 || right < 8) {
                continue;
            }
            double g = left / right;
            if (g < 0.75) g = 0.75;
            if (g > 1.35) g = 1.35;
            gain[i] = gain[i - 1] * g;
            if (gain[i] < 0.75) gain[i] = 0.75;
            if (gain[i] > 1.35) gain[i] = 1.35;
        }
        return gain;
    }
 
    static double overlapMean(Mat bgr, boolean rightEdge, int overlap) {
        int W = bgr.cols();
        int H = bgr.rows();
        int x0 = rightEdge ? W - overlap : 0;
        int x1 = rightEdge ? W : overlap;
        Mat roi = bgr.submat(0, H, x0, x1);
        Mat gray = new Mat();
        Imgproc.cvtColor(roi, gray, Imgproc.COLOR_BGR2GRAY);
        Mat mask = new Mat();
        Imgproc.threshold(gray, mask, INVALID_LUMA, 255, Imgproc.THRESH_BINARY);
        Scalar m = Core.mean(gray, mask);
        roi.release();
        gray.release();
        mask.release();
        return m.val[0];
    }
 
    static Path ensureDecodable(Path requested) {
        Path mp4 = siblingWithExt(requested, ".mp4");
        if (isUsableFile(mp4)) {
            Path a = mp4.toAbsolutePath().normalize();
            Path b = requested.toAbsolutePath().normalize();
            if (!a.equals(b)) {
                System.out.println("Using " + mp4.getFileName() + " (H.264) instead of "
                        + requested.getFileName());
            }
            return mp4;
        }
        if (!isUsableFile(requested)) {
            return requested;
        }
        if (transcodeToH264(requested, mp4) && isUsableFile(mp4)) {
            return mp4;
        }
        System.err.println("Cannot decode " + requested.getFileName()
                + " (PNG-in-MOV). Install ffmpeg on PATH and re-run, or convert:");
        System.err.println("  ffmpeg -y -i " + requested.getFileName()
                + " -c:v libx264 -pix_fmt yuv420p -an " + mp4.getFileName());
        return requested;
    }
 
    static Path siblingWithExt(Path requested, String ext) {
        String name = requested.getFileName().toString();
        int dot = name.lastIndexOf('.');
        String stem = dot >= 0 ? name.substring(0, dot) : name;
        Path dir = requested.toAbsolutePath().getParent();
        if (dir == null) {
            dir = Paths.get(".");
        }
        return dir.resolve(stem + ext);
    }
 
    static boolean isUsableFile(Path path) {
        try {
            return path != null && Files.isRegularFile(path) && Files.size(path) > 0;
        } catch (IOException e) {
            return false;
        }
    }
 
    static boolean transcodeToH264(Path src, Path dst) {
        System.out.println("Transcoding " + src.getFileName() + " -> " + dst.getFileName()
                + " (PNG MOV cannot be decoded by OpenCV FFmpeg/MSMF)");
        ProcessBuilder pb = new ProcessBuilder(
                "ffmpeg", "-hide_banner", "-y",
                "-i", src.toAbsolutePath().toString(),
                "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p",
                "-an", dst.toAbsolutePath().toString());
        pb.inheritIO();
        try {
            int code = pb.start().waitFor();
            if (code == 0 && isUsableFile(dst)) {
                System.out.println("Transcode OK: " + dst.getFileName());
                return true;
            }
            System.err.println("ffmpeg exited with code " + code);
        } catch (IOException e) {
            System.err.println("ffmpeg not found on PATH: " + e.getMessage());
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            System.err.println("ffmpeg transcode interrupted");
        }
        try {
            Files.deleteIfExists(dst);
        } catch (IOException ignored) {
        }
        return false;
    }
 
    static void loadFfmpegPlugin() {
        String[] dllNames = {
            "opencv_videoio_ffmpeg490_64.dll",
            "opencv_videoio_ffmpeg4.dll",
            "opencv_videoio_ffmpeg.dll"
        };
        List<Path> dirs = new ArrayList<>();
        String libPath = System.getProperty("java.library.path", "");
        for (String dir : libPath.split(File.pathSeparator)) {
            if (dir == null || dir.trim().isEmpty()) continue;
            Path p = Paths.get(dir.trim()).toAbsolutePath().normalize();
            dirs.add(p);
            if (p.getParent() != null) {
                dirs.add(p.getParent());
                if (p.getParent().getParent() != null) {
                    dirs.add(p.getParent().getParent().resolve("bin"));
                }
            }
        }
        dirs.add(Paths.get("opencv", "build", "bin").toAbsolutePath());
        dirs.add(Paths.get("..", "opencv", "build", "bin").toAbsolutePath());
 
        for (Path dir : dirs) {
            for (String dll : dllNames) {
                Path candidate = dir.resolve(dll).normalize();
                if (!Files.isRegularFile(candidate)) continue;
                try {
                    System.load(candidate.toString());
                    System.out.println("Loaded FFmpeg videoio plugin: " + candidate);
                    return;
                } catch (Throwable t) {
                    System.err.println("Could not load " + candidate + ": " + t.getMessage());
                }
            }
        }
        System.err.println("FFmpeg videoio plugin not loaded. MSMF will be used for .mov "
                + "and may fail RGB32. Copy opencv_videoio_ffmpeg490_64.dll next to "
                + "opencv_java490.dll or into opencv\\build\\bin on PATH.");
    }
 
    static VideoCapture openVideo(Path path) {
        String file = path.toAbsolutePath().toString();
        int[] apis = { Videoio.CAP_FFMPEG, Videoio.CAP_ANY, Videoio.CAP_MSMF };
        String[] labels = { "FFMPEG", "ANY", "MSMF" };
        for (int a = 0; a < apis.length; a++) {
            for (double convert : new double[]{ (apis[a] == Videoio.CAP_MSMF ? 0 : 1), 0, 1 }) {
                VideoCapture cap = tryOpen(file, apis[a], labels[a], convert);
                if (cap != null) {
                    return cap;
                }
            }
        }
        VideoCapture cap = new VideoCapture();
        cap.open(file);
        if (cap.isOpened()) {
            cap.set(Videoio.CAP_PROP_CONVERT_RGB, 0);
            if (probeFrame(cap)) {
                logBackend(file, cap, "default");
                return cap;
            }
        }
        cap.release();
        return null;
    }
 
    static VideoCapture tryOpen(String file, int api, String label, double convertRgb) {
        VideoCapture cap = new VideoCapture();
        MatOfInt params = new MatOfInt(Videoio.CAP_PROP_CONVERT_RGB, (int) convertRgb);
        try {
            boolean opened = cap.open(file, api, params);
            if (!opened || !cap.isOpened()) {
                cap.release();
                params.release();
                return null;
            }
        } catch (Exception e) {
            cap.release();
            params.release();
            return null;
        }
        params.release();
        cap.set(Videoio.CAP_PROP_CONVERT_RGB, convertRgb);
        if (!probeFrame(cap)) {
            cap.release();
            return null;
        }
        logBackend(file, cap, label + " convertRGB=" + (int) convertRgb);
        return cap;
    }
 
    static boolean probeFrame(VideoCapture cap) {
        Mat probe = new Mat();
        boolean ok = cap.read(probe) && !probe.empty();
        if (ok) {
            Mat bgr = new Mat();
            ok = toBgr(probe, bgr);
            bgr.release();
        }
        probe.release();
        if (!ok) {
            return false;
        }
        cap.set(Videoio.CAP_PROP_POS_FRAMES, 0);
        cap.set(Videoio.CAP_PROP_POS_MSEC, 0);
        return true;
    }
 
    static void logBackend(String file, VideoCapture cap, String requested) {
        String backend = requested;
        try {
            String named = cap.getBackendName();
            if (named != null && !named.isEmpty()) {
                backend = named + " / " + requested;
            }
        } catch (Exception ignored) {
        }
        System.out.println("Opened " + file + " [" + backend + "]");
    }
 
    static boolean toBgr(Mat src, Mat dst) {
        if (src == null || src.empty()) {
            return false;
        }
        int type = src.type();
        if (type == CvType.CV_8UC3) {
            src.copyTo(dst);
            return true;
        }
        if (type == CvType.CV_8UC4) {
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGRA2BGR);
            return !dst.empty();
        }
        if (type == CvType.CV_8UC2) {
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_YUV2BGR_YUY2);
            return !dst.empty();
        }
        if (src.channels() == 1) {
            int h = src.rows();
            int w = src.cols();
            if (h * 2 % 3 == 0) {
                int visH = h * 2 / 3;
                if (visH > 0 && visH * 3 / 2 == h) {
                    try {
                        Imgproc.cvtColor(src, dst, Imgproc.COLOR_YUV2BGR_NV12);
                        if (!dst.empty() && dst.channels() == 3) {
                            return true;
                        }
                    } catch (Exception ignored) {
                    }
                }
            }
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_GRAY2BGR);
            return !dst.empty();
        }
        src.copyTo(dst);
        return !dst.empty();
    }
 
    static boolean readBgr(VideoCapture cap, Mat bgr) {
        if (cap == null || !cap.isOpened()) {
            return false;
        }
        Mat raw = new Mat();
        if (!cap.read(raw) || raw.empty()) {
            raw.release();
            return false;
        }
        boolean ok = toBgr(raw, bgr);
        raw.release();
        return ok && bgr != null && !bgr.empty();
    }
 
    static boolean readOrLoop(VideoCapture[] caps, int i, Path path, Mat frame) {
        if (readBgr(caps[i], frame)) {
            return true;
        }
        caps[i].set(Videoio.CAP_PROP_POS_FRAMES, 0);
        caps[i].set(Videoio.CAP_PROP_POS_MSEC, 0);
        if (readBgr(caps[i], frame)) {
            System.out.println("Video " + i + " looped via seek");
            return true;
        }
        caps[i].release();
        caps[i] = openVideo(path);
        if (caps[i] != null && readBgr(caps[i], frame)) {
            System.out.println("Video " + i + " reopened after loop");
            return true;
        }
        System.err.println("Warning: Could not read frame from video " + i);
        return false;
    }
 
    static byte[] encodeJpeg(Mat frame) {
        MatOfByte buf    = new MatOfByte();
        MatOfInt  params = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 88);
        Imgcodecs.imencode(".jpg", frame, buf, params);
        return buf.toArray();
    }
 
    static void writeFrame(OutputStream out, byte[] jpeg) throws IOException {
        String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                      + jpeg.length + "\r\n\r\n";
        out.write(header.getBytes("UTF-8"));
        out.write(jpeg);
        out.write("\r\n".getBytes("UTF-8"));
        out.flush();
    }
}