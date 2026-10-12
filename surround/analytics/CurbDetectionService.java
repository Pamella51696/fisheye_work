package surround.analytics;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import output.FusedCurbResult;
import perception.curb.CurbPerceptionPipeline;
import surround.runtime.FrameDistributor;
import surround.runtime.FrameSnapshot;

/**
 * Asynchronous curb analytics worker. Failures are isolated from the video pipeline.
 */
public final class CurbDetectionService {

    private final FrameDistributor distributor;
    private final CurbPerceptionPipeline pipeline;
    private final CurbResultFilter filter;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private volatile CurbState state = CurbState.unavailable();
    private volatile FusedCurbResult lastFused;
    private Thread worker;

    public CurbDetectionService(FrameDistributor distributor,
                                CurbPerceptionPipeline pipeline,
                                CurbResultFilter filter) {
        this.distributor = distributor;
        this.pipeline = pipeline;
        this.filter = filter;
    }

    public void start() {
        if (!running.compareAndSet(false, true)) {
            return;
        }
        worker = new Thread(this::loop, "curb-detection");
        worker.setDaemon(true);
        worker.start();
        System.out.println("CurbDetectionService started (async analytics)");
    }

    public void stop() {
        running.set(false);
        if (worker != null) {
            worker.interrupt();
        }
    }

    public CurbState currentState() {
        return state;
    }

    public CurbResultFilter filter() {
        return filter;
    }

    /** Latest full perception output (for dev debug overlay). */
    public FusedCurbResult lastFusedResult() {
        return lastFused;
    }

    private void loop() {
        BlockingQueue<FrameSnapshot> queue = distributor.analyticsQueue();
        while (running.get()) {
            FrameSnapshot snap = null;
            try {
                snap = queue.poll(200, TimeUnit.MILLISECONDS);
                if (snap == null) {
                    continue;
                }
                FusedCurbResult fused = pipeline.process(snap.fisheyeBgr, snap.timestampMs);
                lastFused = fused;
                state = filter.filter(fused);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Throwable t) {
                System.err.println("CurbDetectionService error (video unaffected): " + t.getMessage());
                state = CurbState.unavailable();
            } finally {
                if (snap != null) {
                    snap.release();
                }
            }
        }
    }
}
