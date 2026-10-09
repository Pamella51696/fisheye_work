package surround.vehicle;

/** Fixed steering/yaw for bench testing without a simulator loop. */
public final class ConstantVehicleSignalProvider implements VehicleSignalProvider {

    private final VehicleSignal signal;

    public ConstantVehicleSignalProvider(double steeringDeg, double yawDeg, double velocityMps) {
        this.signal = new VehicleSignal(System.currentTimeMillis(), VehicleSignal.Status.OK,
                steeringDeg, yawDeg, velocityMps);
    }

    @Override
    public VehicleSignal getCurrentSignal() {
        return new VehicleSignal(System.currentTimeMillis(), VehicleSignal.Status.OK,
                signal.steeringAngleDeg, signal.yawAngleDeg, signal.velocityMps);
    }
}
