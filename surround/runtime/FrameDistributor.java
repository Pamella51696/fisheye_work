package surround.runtime;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Fan-out: one latest frame for video consumers, capacity-1 queue for analytics
 * (drops stale frames when curb detection is slow).
 */
public final class FrameDistributor {

    private final AtomicReference<FrameSnapshot> latestVideo = new AtomicReference<>();
    private final AtomicLong videoSequence = new AtomicLong();
    private final BlockingQueue<FrameSnapshot> analyticsQueue = new ArrayBlockingQueue<>(1);
    private final Object videoLock = new Object();

    public void publish(FrameSnapshot snapshot) {
        FrameSnapshot forVideo = FrameSnapshot.cloneSet(
                snapshot.timestampMs, snapshot.sequence, snapshot.fisheyeBgr);
        FrameSnapshot prev = latestVideo.getAndSet(forVideo);
        if (prev != null) {
            prev.release();
        }
        videoSequence.set(snapshot.sequence);
        synchronized (videoLock) {
            videoLock.notifyAll();
        }
        FrameSnapshot forAnalytics = FrameSnapshot.cloneSet(
                snapshot.timestampMs, snapshot.sequence, snapshot.fisheyeBgr);
        FrameSnapshot dropped = analyticsQueue.poll();
        if (dropped != null) {
            dropped.release();
        }
        if (!analyticsQueue.offer(forAnalytics)) {
            forAnalytics.release();
        }
        snapshot.release();
    }

    /** Blocks until a frame newer than {@code lastSequence} is available. */
    public FrameSnapshot awaitVideoFrame(long lastSequence) throws InterruptedException {
        while (true) {
            FrameSnapshot snap = latestVideo.get();
            if (snap != null && snap.sequence > lastSequence) {
                return FrameSnapshot.cloneSet(snap.timestampMs, snap.sequence, snap.fisheyeBgr);
            }
            synchronized (videoLock) {
                snap = latestVideo.get();
                if (snap != null && snap.sequence > lastSequence) {
                    return FrameSnapshot.cloneSet(snap.timestampMs, snap.sequence, snap.fisheyeBgr);
                }
                videoLock.wait(50);
            }
        }
    }

    public BlockingQueue<FrameSnapshot> analyticsQueue() {
        return analyticsQueue;
    }
}
