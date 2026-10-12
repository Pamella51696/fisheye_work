package surround.runtime;

import java.nio.file.Path;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

/**
 * Single reader for all camera files; publishes into {@link FrameDistributor}.
 * Video and analytics consume copies — capture never calls curb detection.
 */
public final class CameraCaptureManager {

    private final Path[] videoFiles;
    private final FrameDistributor distributor;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicLong sequence = new AtomicLong();
    private Thread captureThread;

    public CameraCaptureManager(Path[] videoFiles, FrameDistributor distributor) {
        this.videoFiles = videoFiles;
        this.distributor = distributor;
    }

    public void start() {
        if (!running.compareAndSet(false, true)) {
            return;
        }
        captureThread = new Thread(this::captureLoop, "camera-capture");
        captureThread.setDaemon(true);
        captureThread.start();
    }

    public void stop() {
        running.set(false);
        if (captureThread != null) {
            captureThread.interrupt();
        }
    }

    public FrameDistributor distributor() {
        return distributor;
    }

    private void captureLoop() {
        VideoCapture[] caps = new VideoCapture[videoFiles.length];
        for (int i = 0; i < videoFiles.length; i++) {
            caps[i] = CameraIo.openVideo(videoFiles[i]);
            if (caps[i] == null || !caps[i].isOpened()) {
                System.err.println("CameraCaptureManager: could not open " + videoFiles[i]);
                return;
            }
        }
        Mat[] scratch = new Mat[videoFiles.length];
        for (int i = 0; i < scratch.length; i++) {
            scratch[i] = new Mat();
        }
        try {
            while (running.get()) {
                boolean allReady = true;
                for (int i = 0; i < caps.length; i++) {
                    if (!CameraIo.readOrLoop(caps, i, videoFiles[i], scratch[i])) {
                        allReady = false;
                    }
                }
                if (!allReady) {
                    Thread.sleep(5);
                    continue;
                }
                long seq = sequence.incrementAndGet();
                long ts = System.currentTimeMillis();
                FrameSnapshot snap = FrameSnapshot.cloneSet(ts, seq, scratch);
                distributor.publish(snap);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            for (VideoCapture c : caps) {
                if (c != null) {
                    c.release();
                }
            }
            for (Mat m : scratch) {
                if (m != null) {
                    m.release();
                }
            }
        }
    }
}
