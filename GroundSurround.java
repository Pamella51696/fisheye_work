import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Top-down surround view on one ground plane.
 *
 *   4 camera frames
 *     → fisheye correction (K, D)
 *     → metric ground canvas
 *     → each camera placed on the common plane Z = 0
 *     → blend by heading
 *     → surround view
 *
 * An output pixel is a point on the ground. That point is projected into every
 * camera with the fisheye model, sampled, and mixed. Front, left, right, and
 * rear therefore share one vehicle frame instead of four separate pictures.
 */
final class GroundSurround {

    static final int SIZE = 640;
    /** Half-width of the ground canvas, in meters. The image covers a square. */
    static final double HALF_EXTENT_M = 6.0;
    static final double VEHICLE_HALF_LENGTH_M = 2.25;
    static final double VEHICLE_HALF_WIDTH_M = 0.95;

    /**
     * Optical center of each camera in the vehicle frame, meters.
     * X forward, Y right, Z up. Ground is Z = 0.
     * Order matches {@link VideoStreamingServer#CAM_ROLE}: left, front, right, rear.
     */
    static final double[][] CAM_POS_M = {
            { 0.30, -0.95, 1.05 },
            { 2.15,  0.00, 0.65 },
            { 0.30,  0.95, 1.05 },
            {-2.15,  0.00, 0.85 }
    };

    private static final double OWN_DEG = 35.0;
    private static final double DROP_DEG = 55.0;
    private static final Scalar BODY = new Scalar(32, 34, 38);

    private final Mat[] map1 = new Mat[4];
    private final Mat[] map2 = new Mat[4];
    private final Mat[] weight = new Mat[4];
    private final double[][] signature = new double[4][];

    static boolean isSelfTestArgs(String[] args) {
        return has(args, "--surround-self-test");
    }

    static boolean isPreviewArgs(String[] args) {
        return has(args, "--surround-preview");
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

    /** One surround image from four BGR frames. Uses saved calibration when present. */
    Mat render(Mat[] frames) {
        Lens[] lenses = new Lens[4];
        for (int i = 0; i < 4; i++) {
            lenses[i] = Lens.fromCalibration(i, frames[i].cols(), frames[i].rows());
        }
        return render(frames, lenses);
    }

    Mat render(Mat[] frames, Lens[] lenses) {
        ensureMaps(lenses);
        Mat acc = Mat.zeros(SIZE, SIZE, CvType.CV_32FC3);
        Mat wsum = Mat.zeros(SIZE, SIZE, CvType.CV_32FC1);
        Mat[] warped = new Mat[4];
        for (int i = 0; i < 4; i++) {
            warped[i] = new Mat();
            Imgproc.remap(frames[i], warped[i], map1[i], map2[i], Imgproc.INTER_LINEAR,
                    Core.BORDER_CONSTANT, new Scalar(0, 0, 0));
            accumulate(warped[i], weight[i], acc, wsum);
            warped[i].release();
        }
        Mat out = normalize(acc, wsum);
        paintVehicle(out);
        acc.release();
        wsum.release();
        return out;
    }

    private void ensureMaps(Lens[] lenses) {
        boolean same = true;
        for (int i = 0; i < 4; i++) {
            double[] sig = lenses[i].signature();
            if (!Arrays.equals(sig, signature[i])) {
                same = false;
                break;
            }
        }
        if (same && map1[0] != null && !map1[0].empty()) {
            return;
        }
        double mPerPx = (2.0 * HALF_EXTENT_M) / SIZE;
        for (int cam = 0; cam < 4; cam++) {
            Mat mapX = new Mat(SIZE, SIZE, CvType.CV_32FC1);
            Mat mapY = new Mat(SIZE, SIZE, CvType.CV_32FC1);
            Mat w = new Mat(SIZE, SIZE, CvType.CV_32FC1);
            float[] rowX = new float[SIZE];
            float[] rowY = new float[SIZE];
            float[] rowW = new float[SIZE];
            for (int row = 0; row < SIZE; row++) {
                double x = (SIZE / 2.0 - (row + 0.5)) * mPerPx;
                for (int col = 0; col < SIZE; col++) {
                    double y = ((col + 0.5) - SIZE / 2.0) * mPerPx;
                    float[] uv = projectGround(lenses[cam], x, y);
                    if (uv == null || insideVehicle(x, y)) {
                        rowX[col] = -1f;
                        rowY[col] = -1f;
                        rowW[col] = 0f;
                    } else {
                        rowX[col] = uv[0];
                        rowY[col] = uv[1];
                        rowW[col] = blendWeight(cam, x, y);
                    }
                }
                mapX.put(row, 0, rowX);
                mapY.put(row, 0, rowY);
                w.put(row, 0, rowW);
            }
            if (map1[cam] == null) {
                map1[cam] = new Mat();
            }
            if (map2[cam] == null) {
                map2[cam] = new Mat();
            }
            Imgproc.convertMaps(mapX, mapY, map1[cam], map2[cam], CvType.CV_16SC2, false);
            mapX.release();
            mapY.release();
            if (weight[cam] != null) {
                weight[cam].release();
            }
            weight[cam] = w;
            signature[cam] = lenses[cam].signature();
        }
    }

    /** Fisheye pixel that sees ground point (x, y), or null when the camera cannot. */
    static float[] projectGround(Lens lens, double x, double y) {
        double dx = x - lens.x;
        double dy = y - lens.y;
        double dz = -lens.z;
        double xc = lens.r[0] * dx + lens.r[3] * dy + lens.r[6] * dz;
        double yc = lens.r[1] * dx + lens.r[4] * dy + lens.r[7] * dz;
        double zc = lens.r[2] * dx + lens.r[5] * dy + lens.r[8] * dz;
        if (zc <= 1e-4) {
            return null;
        }
        float[] uv = CalibrationManager.projectFisheye(
                xc, yc, zc, lens.k, lens.k1, lens.k2, lens.k3, lens.k4);
        if (uv == null) {
            return null;
        }
        if (uv[0] < 1 || uv[1] < 1 || uv[0] >= lens.srcW - 1 || uv[1] >= lens.srcH - 1) {
            return null;
        }
        return uv;
    }

    static boolean insideVehicle(double x, double y) {
        return Math.abs(x) < VEHICLE_HALF_LENGTH_M && Math.abs(y) < VEHICLE_HALF_WIDTH_M;
    }

    /** 1 in this camera's quadrant, fading to 0 across the diagonal seams. */
    static float blendWeight(int cam, double x, double y) {
        double az = Math.atan2(y, x);
        double yaw = Math.toRadians(VideoStreamingServer.CAM_YAW_DEG[cam]);
        double deg = Math.abs(Math.toDegrees(wrap(az - yaw)));
        if (deg >= DROP_DEG) {
            return 0f;
        }
        double ang = 1.0;
        if (deg > OWN_DEG) {
            double t = (DROP_DEG - deg) / (DROP_DEG - OWN_DEG);
            ang = t * t * (3.0 - 2.0 * t);
        }
        return (float) ang;
    }

    static int[] groundPixel(double x, double y) {
        double mPerPx = (2.0 * HALF_EXTENT_M) / SIZE;
        int col = (int) Math.round(SIZE / 2.0 + y / mPerPx - 0.5);
        int row = (int) Math.round(SIZE / 2.0 - x / mPerPx - 0.5);
        return new int[] { col, row };
    }

    private static void accumulate(Mat warped, Mat w, Mat acc, Mat wsum) {
        Mat f = new Mat();
        warped.convertTo(f, CvType.CV_32FC3);
        List<Mat> ch = new ArrayList<>(3);
        Core.split(f, ch);
        for (int i = 0; i < 3; i++) {
            Core.multiply(ch.get(i), w, ch.get(i));
        }
        Core.merge(ch, f);
        Core.add(acc, f, acc);
        Core.add(wsum, w, wsum);
        for (Mat c : ch) {
            c.release();
        }
        f.release();
    }

    private static Mat normalize(Mat acc, Mat wsum) {
        Mat low = new Mat();
        Imgproc.threshold(wsum, low, 0.02, 255, Imgproc.THRESH_BINARY_INV);
        Mat low8 = new Mat();
        low.convertTo(low8, CvType.CV_8U);
        Mat w3 = new Mat();
        Core.merge(Arrays.asList(wsum, wsum, wsum), w3);
        w3.setTo(new Scalar(1, 1, 1), low8);
        Mat divided = new Mat();
        Core.divide(acc, w3, divided);
        Mat out = new Mat();
        divided.convertTo(out, CvType.CV_8UC3);
        out.setTo(new Scalar(0, 0, 0), low8);
        low.release();
        low8.release();
        w3.release();
        divided.release();
        return out;
    }

    private static void paintVehicle(Mat bgr) {
        int[] a = groundPixel(VEHICLE_HALF_LENGTH_M, -VEHICLE_HALF_WIDTH_M);
        int[] b = groundPixel(-VEHICLE_HALF_LENGTH_M, VEHICLE_HALF_WIDTH_M);
        Imgproc.rectangle(bgr,
                new org.opencv.core.Point(a[0], a[1]),
                new org.opencv.core.Point(b[0], b[1]),
                BODY, Imgproc.FILLED);
        Imgproc.rectangle(bgr,
                new org.opencv.core.Point(a[0], a[1]),
                new org.opencv.core.Point(b[0], b[1]),
                new Scalar(90, 96, 104), 2);
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
     * Paints each camera a solid color and checks that the four headings land
     * in the matching quadrant of the ground plane.
     */
    static int selfTest() {
        int w = 640;
        int h = 480;
        Lens[] lenses = new Lens[4];
        Mat[] frames = new Mat[4];
        Scalar[] color = {
                new Scalar(210, 40, 40),
                new Scalar(40, 180, 40),
                new Scalar(40, 40, 210),
                new Scalar(40, 180, 200)
        };
        for (int i = 0; i < 4; i++) {
            lenses[i] = Lens.nominal(i, w, h);
            frames[i] = new Mat(h, w, CvType.CV_8UC3, color[i]);
        }
        GroundSurround surround = new GroundSurround();
        Mat out = surround.render(frames, lenses);
        boolean ok = true;
        ok &= expect(out, "front", 4.2, 0.0, color[1]);
        ok &= expect(out, "right", 0.0, 4.2, color[2]);
        ok &= expect(out, "rear", -4.2, 0.0, color[3]);
        ok &= expect(out, "left", 0.0, -4.2, color[0]);
        int[] origin = groundPixel(0, 0);
        double[] body = out.get(origin[1], origin[0]);
        if (body == null || Math.abs(body[0] - BODY.val[0]) > 2) {
            System.err.println("surround self-test: vehicle body missing at center");
            ok = false;
        }
        org.opencv.imgcodecs.Imgcodecs.imwrite("/tmp/surround_selftest.jpg", out);
        for (Mat f : frames) {
            f.release();
        }
        out.release();
        if (!ok) {
            System.err.println("surround self-test FAILED");
            return 1;
        }
        System.out.println("surround self-test OK");
        return 0;
    }

    private static boolean expect(Mat out, String name, double x, double y, Scalar bgr) {
        int[] px = groundPixel(x, y);
        double[] got = out.get(px[1], px[0]);
        if (got == null) {
            System.err.println("surround self-test: " + name + " pixel missing");
            return false;
        }
        for (int c = 0; c < 3; c++) {
            if (Math.abs(got[c] - bgr.val[c]) > 8) {
                System.err.printf(Locale.US,
                        "surround self-test: %s at (%.1f, %.1f) px (%d,%d) got [%.0f %.0f %.0f] want [%.0f %.0f %.0f]%n",
                        name, x, y, px[0], px[1], got[0], got[1], got[2],
                        bgr.val[0], bgr.val[1], bgr.val[2]);
                return false;
            }
        }
        return true;
    }

    /** Fisheye plus the mount that places this camera on the vehicle. */
    static final class Lens {
        final double[] r;
        final double x;
        final double y;
        final double z;
        final double[] k;
        final double k1, k2, k3, k4;
        final int srcW;
        final int srcH;

        Lens(double[] r, double x, double y, double z, double[] k,
             double k1, double k2, double k3, double k4, int srcW, int srcH) {
            this.r = r;
            this.x = x;
            this.y = y;
            this.z = z;
            this.k = k;
            this.k1 = k1;
            this.k2 = k2;
            this.k3 = k3;
            this.k4 = k4;
            this.srcW = srcW;
            this.srcH = srcH;
        }

        static Lens nominal(int cam, int srcW, int srcH) {
            double f = VideoStreamingServer.fisheyeFocal(
                    srcW, srcH, VideoStreamingServer.INPUT_FISHEYE_FOV_DEG);
            double[] poseR = VideoStreamingServer.cameraToVehicle(
                    VideoStreamingServer.CAM_YAW_DEG[cam],
                    VideoStreamingServer.CAM_PITCH_DEG[cam],
                    VideoStreamingServer.CAM_ROLL_DEG[cam]);
            return new Lens(poseR,
                    CAM_POS_M[cam][0], CAM_POS_M[cam][1], CAM_POS_M[cam][2],
                    new double[] { f, f, srcW * 0.5, srcH * 0.5 },
                    0, 0, 0, 0, srcW, srcH);
        }

        static Lens fromCalibration(int cam, int srcW, int srcH) {
            CalibrationManager.CameraModel model =
                    CalibrationManager.load(VideoStreamingServer.CAM_ROLE[cam]);
            double[] pose = CalibrationManager.mountDegreesForStitch(
                    cam, model, Double.NaN, Double.NaN);
            double[] poseR = VideoStreamingServer.cameraToVehicle(pose[0], pose[1], pose[2]);
            boolean use = CalibrationManager.stableIntrinsics(model);
            double f = VideoStreamingServer.fisheyeFocal(
                    srcW, srcH, VideoStreamingServer.INPUT_FISHEYE_FOV_DEG);
            double[] k = use
                    ? model.scaledK(srcW, srcH)
                    : new double[] {
                        f, f,
                        srcW * VideoStreamingServer.FISHEYE_CX,
                        srcH * VideoStreamingServer.FISHEYE_CY };
            return new Lens(poseR,
                    CAM_POS_M[cam][0], CAM_POS_M[cam][1], CAM_POS_M[cam][2],
                    k,
                    use ? model.k1 : 0,
                    use ? model.k2 : 0,
                    use ? model.k3 : 0,
                    use ? model.k4 : 0,
                    srcW, srcH);
        }

        double[] signature() {
            return new double[] {
                x, y, z, k[0], k[1], k[2], k[3], k1, k2, k3, k4,
                r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], srcW, srcH
            };
        }
    }
}
