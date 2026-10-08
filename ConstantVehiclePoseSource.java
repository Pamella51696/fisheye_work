public final class ConstantVehiclePoseSource implements VehiclePoseSource {
    private final VehiclePose pose;

    public ConstantVehiclePoseSource(double steeringDeg, double headingDeg) {
        this.pose = new VehiclePose(steeringDeg, headingDeg);
    }

    @Override
    public VehiclePose sample(long timeNanos) {
        return pose;
    }
}
