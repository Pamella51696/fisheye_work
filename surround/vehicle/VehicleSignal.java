package surround.vehicle;

/** Steering / yaw snapshot for wheel-alignment overlay. */
public final class VehicleSignal {

    public enum Status {
        OK,
        UNAVAILABLE
    }

    public final long timestampMs;
    public final Status status;
    public final double steeringAngleDeg;
    public final double yawAngleDeg;
    public final double velocityMps;

    public VehicleSignal(long timestampMs,
                         Status status,
                         double steeringAngleDeg,
                         double yawAngleDeg,
                         double velocityMps) {
        this.timestampMs = timestampMs;
        this.status = status;
        this.steeringAngleDeg = steeringAngleDeg;
        this.yawAngleDeg = yawAngleDeg;
        this.velocityMps = velocityMps;
    }

    public static VehicleSignal unavailable() {
        return new VehicleSignal(System.currentTimeMillis(), Status.UNAVAILABLE,
                0.0, 0.0, 0.0);
    }
}
