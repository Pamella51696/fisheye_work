package surround.vehicle;

/** Steering and heading snapshot (UDP wire format uses heading as yaw). */
public final class VehiclePose {

    public final double steeringDeg;
    public final double headingDeg;

    public VehiclePose(double steeringDeg, double headingDeg) {
        this.steeringDeg = steeringDeg;
        this.headingDeg = headingDeg;
    }
}
