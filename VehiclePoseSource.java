/** Supplies steering / heading at the rate the UDP publisher polls. */
public interface VehiclePoseSource {
    /**
     * @param timeNanos monotonic clock ({@link System#nanoTime()}) for simulation
     */
    VehiclePose sample(long timeNanos);
}
