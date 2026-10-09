package surround.runtime;

import org.opencv.core.Mat;

/** Immutable synchronized multi-camera frame set (caller must {@link #release()}). */
public final class FrameSnapshot {

    public final long timestampMs;
    public final long sequence;
    public final Mat[] fisheyeBgr;

    public FrameSnapshot(long timestampMs, long sequence, Mat[] fisheyeBgr) {
        this.timestampMs = timestampMs;
        this.sequence = sequence;
        this.fisheyeBgr = fisheyeBgr;
    }

    public static FrameSnapshot cloneSet(long timestampMs, long sequence, Mat[] source) {
        Mat[] copies = new Mat[source.length];
        for (int i = 0; i < source.length; i++) {
            copies[i] = new Mat();
            source[i].copyTo(copies[i]);
        }
        return new FrameSnapshot(timestampMs, sequence, copies);
    }

    public void release() {
        if (fisheyeBgr == null) {
            return;
        }
        for (Mat m : fisheyeBgr) {
            if (m != null) {
                m.release();
            }
        }
    }
}
