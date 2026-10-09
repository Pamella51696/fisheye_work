package surround.vehicle;

/** Bridges {@link VehicleSignalProvider} to outbound UDP publisher sampling. */
public final class ProviderVehiclePoseSource implements VehiclePoseSource {

    private final VehicleSignalProvider provider;

    public ProviderVehiclePoseSource(VehicleSignalProvider provider) {
        this.provider = provider;
    }

    @Override
    public VehiclePose sample(long timeNanos) {
        VehicleSignal s = provider.getCurrentSignal();
        if (s.status != VehicleSignal.Status.OK) {
            return new VehiclePose(0.0, 0.0);
        }
        return new VehiclePose(s.steeringAngleDeg, s.yawAngleDeg);
    }
}
