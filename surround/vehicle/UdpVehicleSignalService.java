package surround.vehicle;

import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * Vehicle-signal gateway: simulator (dev), UDP ingress (vehicle/Android bus), or constant pose.
 * Optional UDP egress mirrors the active provider to the legacy 16-byte Android packet.
 */
public final class UdpVehicleSignalService implements VehicleSignalProvider {

    private final VehicleSignalSimulator simulator;
    private final VehicleSignalConfig config;
    private volatile VehicleSignalProvider activeProvider;
    private UdpInboundPoseListener inboundListener;
    private UdpVehiclePosePublisher outboundPublisher;

    public UdpVehicleSignalService(VehicleSignalSimulator simulator, VehicleSignalConfig config) {
        this.simulator = simulator;
        this.config = config;
        this.activeProvider = simulator;
    }

    public void start() {
        switch (config.poseMode) {
            case SIM:
                simulator.start();
                activeProvider = simulator;
                System.out.println("Vehicle signals: simulator (POST /api/vehicle/simulate)");
                break;
            case CONSTANT:
                activeProvider = new ConstantVehicleSignalProvider(
                        config.constantSteeringDeg, config.constantYawDeg, 0.0);
                System.out.println("Vehicle signals: constant steering="
                        + config.constantSteeringDeg + "° yaw=" + config.constantYawDeg + "°");
                break;
            case UDP:
                try {
                    inboundListener = new UdpInboundPoseListener(config.udpListenPort);
                    inboundListener.start();
                    activeProvider = inboundListener;
                    System.out.println("Vehicle signals: UDP ingress on port " + config.udpListenPort);
                } catch (SocketException e) {
                    System.err.println("UDP ingress failed; vehicle UNAVAILABLE: " + e.getMessage());
                    activeProvider = null;
                }
                break;
            default:
                simulator.start();
                activeProvider = simulator;
        }
        startEgressIfConfigured();
    }

    private void startEgressIfConfigured() {
        if (config.udpTargetHost == null || config.udpTargetHost.isEmpty()) {
            return;
        }
        VehicleSignalProvider src = activeProvider != null ? activeProvider : simulator;
        try {
            outboundPublisher = new UdpVehiclePosePublisher(
                    new ProviderVehiclePoseSource(src),
                    config.udpTargetHost,
                    config.udpTargetPort,
                    config.udpEgressHz);
            outboundPublisher.start();
        } catch (UnknownHostException | SocketException e) {
            System.err.println("UDP egress disabled: " + e.getMessage());
        }
    }

    public void stop() {
        simulator.stop();
        if (inboundListener != null) {
            inboundListener.close();
        }
        if (outboundPublisher != null) {
            outboundPublisher.close();
        }
    }

    public VehicleSignalSimulator simulator() {
        return simulator;
    }

    public void useSimulator() {
        simulator.start();
        activeProvider = simulator;
    }

    public VehicleSignalConfig config() {
        return config;
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
