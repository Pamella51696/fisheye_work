package surround.runtime;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

/** Shared video file capture helpers (single capture manager + legacy paths). */
public final class CameraIo {

    private CameraIo() {
    }

    public static VideoCapture openVideo(Path path) {
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
                return cap;
            }
        }
        cap.release();
        return null;
    }

    private static VideoCapture tryOpen(String file, int api, String label, double convertRgb) {
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
        return cap;
    }

    private static boolean probeFrame(VideoCapture cap) {
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

    public static boolean toBgr(Mat src, Mat dst) {
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
            Imgproc.cvtColor(src, dst, Imgproc.COLOR_GRAY2BGR);
            return !dst.empty();
        }
        src.copyTo(dst);
        return !dst.empty();
    }

    public static boolean readBgr(VideoCapture cap, Mat bgr) {
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
        return ok && !bgr.empty();
    }

    public static boolean readOrLoop(VideoCapture[] caps, int i, Path path, Mat frame) {
        if (readBgr(caps[i], frame)) {
            return true;
        }
        caps[i].set(Videoio.CAP_PROP_POS_FRAMES, 0);
        caps[i].set(Videoio.CAP_PROP_POS_MSEC, 0);
        if (readBgr(caps[i], frame)) {
            return true;
        }
        caps[i].release();
        caps[i] = openVideo(path);
        return caps[i] != null && readBgr(caps[i], frame);
    }

    public static Path siblingWithExt(Path requested, String ext) {
        String name = requested.getFileName().toString();
        int dot = name.lastIndexOf('.');
        String stem = dot >= 0 ? name.substring(0, dot) : name;
        Path dir = requested.toAbsolutePath().getParent();
        if (dir == null) {
            dir = Paths.get(".");
        }
        return dir.resolve(stem + ext);
    }

    public static boolean isUsableFile(Path path) {
        try {
            return path != null && Files.isRegularFile(path) && Files.size(path) > 0;
        } catch (IOException e) {
            return false;
        }
    }

    public static Path ensureDecodable(Path requested) {
        Path mp4 = siblingWithExt(requested, ".mp4");
        if (isUsableFile(mp4)) {
            return mp4;
        }
        if (!isUsableFile(requested)) {
            return requested;
        }
        return requested;
    }

    public static void loadFfmpegPlugin() {
        String[] dllNames = {
            "opencv_videoio_ffmpeg490_64.dll",
            "opencv_videoio_ffmpeg4.dll",
            "opencv_videoio_ffmpeg.dll"
        };
        List<Path> dirs = new ArrayList<>();
        String libPath = System.getProperty("java.library.path", "");
        for (String dir : libPath.split(java.io.File.pathSeparator)) {
            if (dir == null || dir.trim().isEmpty()) {
                continue;
            }
            dirs.add(Paths.get(dir.trim()).toAbsolutePath().normalize());
        }
        dirs.add(Paths.get("opencv", "build", "bin").toAbsolutePath());
        for (Path dir : dirs) {
            for (String dll : dllNames) {
                Path candidate = dir.resolve(dll).normalize();
                if (!Files.isRegularFile(candidate)) {
                    continue;
                }
                try {
                    System.load(candidate.toString());
                    System.out.println("Loaded FFmpeg videoio plugin: " + candidate);
                    return;
                } catch (Throwable ignored) {
                }
            }
        }
    }
}
