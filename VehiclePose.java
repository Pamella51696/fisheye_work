/** Latest vehicle orientation for the phone UI (packet contract). */
public final class VehiclePose {
    public final double steeringDeg;
    public final double headingDeg;

    public VehiclePose(double steeringDeg, double headingDeg) {
        this.steeringDeg = steeringDeg;
        this.headingDeg = headingDeg;
    }
}
