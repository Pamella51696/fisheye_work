/** Quick loopback check for the 16-byte LE packet layout. */
public final class UdpPoseSmokeTest {
    public static void main(String[] args) throws Exception {
        MiddlewareConfig config = MiddlewareConfig.fromEnvironmentAndArgs(
                new String[] {
                    "--udp-target", "127.0.0.1",
                    "--udp-port", "45454",
                    "--pose-mode", "constant",
                    "--steering-deg", "5.5",
                    "--heading-deg", "90.25"
                },
                9090);
        try (UdpVehiclePosePublisher pub = MiddlewareLauncher.startPoseUdp(config)) {
            Thread.sleep(150);
        }
    }
}
