import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * Wires vehicle pose sourcing to the UDP packet contract expected by the Android receiver.
 */
public final class MiddlewareLauncher {
    private MiddlewareLauncher() {}

    public static UdpVehiclePosePublisher startPoseUdp(MiddlewareConfig config)
            throws UnknownHostException, SocketException {
        if (config.udpTargetHost == null || config.udpTargetHost.isEmpty()) {
            System.out.println("UDP pose disabled (set --udp-target <phone-ip> or UDP_TARGET_HOST)");
            return null;
        }
        VehiclePoseSource source = createPoseSource(config);
        UdpVehiclePosePublisher publisher =
                new UdpVehiclePosePublisher(source, config.udpTargetHost, config.udpPort, config.udpHz);
        publisher.start();
        return publisher;
    }

    static VehiclePoseSource createPoseSource(MiddlewareConfig config) {
        String mode = config.poseMode == null ? "sim" : config.poseMode.toLowerCase();
        switch (mode) {
            case "constant":
                return new ConstantVehiclePoseSource(config.constantSteeringDeg, config.constantHeadingDeg);
            case "live":
                MiddlewarePoseBus.seed(config.constantSteeringDeg, config.constantHeadingDeg);
                return MiddlewarePoseBus.liveSource();
            case "sim":
            case "simulated":
                return new SimulatedVehiclePoseSource(config.constantHeadingDeg, 12.0);
            default:
                System.err.println("Unknown pose mode '" + config.poseMode + "', using sim");
                return new SimulatedVehiclePoseSource(0.0, 12.0);
        }
    }
}
