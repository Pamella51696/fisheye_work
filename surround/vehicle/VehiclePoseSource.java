package surround.vehicle;

/** Supplies steering / heading when publishing outbound UDP. */
public interface VehiclePoseSource {

    /**
     * @param timeNanos monotonic clock ({@link System#nanoTime()}) for simulation
     */
    VehiclePose sample(long timeNanos);
}
