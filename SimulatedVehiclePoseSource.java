/**
 * Demo pose when no CAN / GPS is wired: slow turn with modest steering for UI testing.
 */
public final class SimulatedVehiclePoseSource implements VehiclePoseSource {
    private final double headingOffsetDeg;
    private final double steeringAmplitudeDeg;

    public SimulatedVehiclePoseSource(double headingOffsetDeg, double steeringAmplitudeDeg) {
        this.headingOffsetDeg = headingOffsetDeg;
        this.steeringAmplitudeDeg = steeringAmplitudeDeg;
    }

    @Override
    public VehiclePose sample(long timeNanos) {
        double t = timeNanos * 1e-9;
        double headingDeg = headingOffsetDeg + 30.0 * Math.sin(t * 0.08);
        double steeringDeg = steeringAmplitudeDeg * Math.sin(t * 0.12);
        return new VehiclePose(steeringDeg, headingDeg);
    }
}
