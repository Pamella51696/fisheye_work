/**
 * Single injection point for real vehicle data into the UDP pose stream.
 * Call {@link #update(double, double)} from CAN readers, sim bridges, etc.
 */
public final class MiddlewarePoseBus {
    private static final LatestVehiclePoseSource LIVE =
            new LatestVehiclePoseSource(0.0, 0.0);

    private MiddlewarePoseBus() {}

    public static VehiclePoseSource liveSource() {
        return LIVE;
    }

    public static void update(double steeringDeg, double headingDeg) {
        LIVE.update(steeringDeg, headingDeg);
    }

    public static void seed(double steeringDeg, double headingDeg) {
        LIVE.update(steeringDeg, headingDeg);
    }
}
