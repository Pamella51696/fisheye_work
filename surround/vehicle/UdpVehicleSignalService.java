package surround.vehicle;

/**
 * Vehicle-signal gateway: real UDP later; simulator is the default provider for development.
 */
public final class UdpVehicleSignalService implements VehicleSignalProvider {

    private final VehicleSignalSimulator simulator;
    private volatile VehicleSignalProvider activeProvider;

    public UdpVehicleSignalService(VehicleSignalSimulator simulator) {
        this.simulator = simulator;
        this.activeProvider = simulator;
    }

    public void start() {
        simulator.start();
        System.out.println("UdpVehicleSignalService: using simulator (replace with RealUdpProvider later)");
    }

    public void stop() {
        simulator.stop();
    }

    public VehicleSignalSimulator simulator() {
        return simulator;
    }

    public void useSimulator() {
        activeProvider = simulator;
    }

    @Override
    public VehicleSignal getCurrentSignal() {
        VehicleSignalProvider p = activeProvider;
        if (p == null) {
            return VehicleSignal.unavailable();
        }
        try {
            return p.getCurrentSignal();
        } catch (Throwable t) {
            return VehicleSignal.unavailable();
        }
    }
}
