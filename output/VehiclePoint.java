package output;

/** Vehicle frame: X forward, Y right, Z up (meters). */
public final class VehiclePoint {
    public final double x;
    public final double y;
    public final double z;

    public VehiclePoint(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}
