package surround.vehicle;

/** Thread-safe latest pose from CAN, UDP ingress, or manual injection. */
public final class LatestVehiclePoseSource implements VehiclePoseSource {

    private volatile VehiclePose latest;

    public LatestVehiclePoseSource(double steeringDeg, double headingDeg) {
        this.latest = new VehiclePose(steeringDeg, headingDeg);
    }

    public void update(double steeringDeg, double headingDeg) {
        this.latest = new VehiclePose(steeringDeg, headingDeg);
    }

    @Override
    public VehiclePose sample(long timeNanos) {
        return latest;
    }
}
