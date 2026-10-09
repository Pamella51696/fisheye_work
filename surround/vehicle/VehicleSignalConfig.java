package surround.vehicle;

import java.util.ArrayList;
import java.util.List;

/** CLI + environment configuration for vehicle pose ingress/egress and signal WebSocket. */
public final class VehicleSignalConfig {

    public enum PoseMode {
        SIM,
        UDP,
        CONSTANT
    }

    public final int httpPort;
    public final int signalWebSocketPort;
    public final double signalWebSocketHz;
    public final PoseMode poseMode;
    public final int udpListenPort;
    public final String udpTargetHost;
    public final int udpTargetPort;
    public final double udpEgressHz;
    public final double constantSteeringDeg;
    public final double constantYawDeg;

    private VehicleSignalConfig(
            int httpPort,
            int signalWebSocketPort,
            double signalWebSocketHz,
            PoseMode poseMode,
            int udpListenPort,
            String udpTargetHost,
            int udpTargetPort,
            double udpEgressHz,
            double constantSteeringDeg,
            double constantYawDeg) {
        this.httpPort = httpPort;
        this.signalWebSocketPort = signalWebSocketPort;
        this.signalWebSocketHz = signalWebSocketHz;
        this.poseMode = poseMode;
        this.udpListenPort = udpListenPort;
        this.udpTargetHost = udpTargetHost;
        this.udpTargetPort = udpTargetPort;
        this.udpEgressHz = udpEgressHz;
        this.constantSteeringDeg = constantSteeringDeg;
        this.constantYawDeg = constantYawDeg;
    }

    public static VehicleSignalConfig parse(String[] args, int defaultHttpPort) {
        int httpPort = defaultHttpPort;
        int wsPort = parseIntEnv("SIGNAL_WS_PORT", 9091);
        double wsHz = parseDoubleEnv("SIGNAL_WS_HZ", 10.0);
        PoseMode mode = parseMode(envOrDefault("VEHICLE_POSE_MODE", "sim"));
        int listenPort = parseIntEnv("UDP_LISTEN_PORT", 0);
        String udpTarget = envOrNull("UDP_TARGET_HOST");
        int udpPort = parseIntEnv("UDP_PORT", 45454);
        double udpHz = parseDoubleEnv("UDP_HZ", 20.0);
        double steering = parseDoubleEnv("VEHICLE_STEERING_DEG", 0.0);
        double yaw = parseDoubleEnv("VEHICLE_YAW_DEG", parseDoubleEnv("VEHICLE_HEADING_DEG", 0.0));

        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            if ("--signal-ws-port".equals(a) && i + 1 < args.length) {
                wsPort = Integer.parseInt(args[++i]);
            } else if ("--no-signal-ws".equals(a)) {
                wsPort = 0;
            } else if ("--signal-ws-hz".equals(a) && i + 1 < args.length) {
                wsHz = Double.parseDouble(args[++i]);
            } else if ("--vehicle-pose-mode".equals(a) && i + 1 < args.length) {
                mode = parseMode(args[++i]);
            } else if ("--udp-listen-port".equals(a) && i + 1 < args.length) {
                listenPort = Integer.parseInt(args[++i]);
            } else if ("--udp-target".equals(a) && i + 1 < args.length) {
                udpTarget = args[++i];
            } else if ("--udp-port".equals(a) && i + 1 < args.length) {
                udpPort = Integer.parseInt(args[++i]);
            } else if ("--udp-hz".equals(a) && i + 1 < args.length) {
                udpHz = Double.parseDouble(args[++i]);
            } else if ("--steering-deg".equals(a) && i + 1 < args.length) {
                steering = Double.parseDouble(args[++i]);
            } else if ("--yaw-deg".equals(a) && i + 1 < args.length) {
                yaw = Double.parseDouble(args[++i]);
            } else if ("--help".equals(a) || "-h".equals(a)) {
                printUsage();
            } else if (a.startsWith("--")) {
                System.err.println("Unknown option: " + a);
            } else {
                try {
                    httpPort = Integer.parseInt(a);
                } catch (NumberFormatException ignored) {
                    System.err.println("Ignoring unrecognized argument: " + a);
                }
            }
        }

        if (mode == PoseMode.UDP && listenPort <= 0) {
            listenPort = parseIntEnv("UDP_LISTEN_PORT", 45454);
        }

        return new VehicleSignalConfig(httpPort, wsPort, wsHz, mode, listenPort,
                udpTarget, udpPort, udpHz, steering, yaw);
    }

    /** Args with flags removed (for callers that only need the HTTP port). */
    public static String[] positionalOnly(String[] args) {
        List<String> out = new ArrayList<>();
        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            if (a.startsWith("--")) {
                if ("--no-signal-ws".equals(a)) {
                    continue;
                }
                if (i + 1 < args.length && needsValue(a)) {
                    i++;
                }
                continue;
            }
            out.add(a);
        }
        return out.toArray(new String[0]);
    }

    private static boolean needsValue(String flag) {
        return !"--no-signal-ws".equals(flag);
    }

    private static PoseMode parseMode(String raw) {
        if (raw == null) {
            return PoseMode.SIM;
        }
        switch (raw.toLowerCase()) {
            case "udp":
            case "live":
                return PoseMode.UDP;
            case "constant":
                return PoseMode.CONSTANT;
            case "sim":
            case "simulated":
            default:
                return PoseMode.SIM;
        }
    }

    private static String envOrNull(String key) {
        String v = System.getenv(key);
        if (v == null || v.isEmpty()) {
            return null;
        }
        return v.trim();
    }

    private static String envOrDefault(String key, String def) {
        String v = envOrNull(key);
        return v == null ? def : v;
    }

    private static int parseIntEnv(String key, int def) {
        String v = envOrNull(key);
        if (v == null) {
            return def;
        }
        try {
            return Integer.parseInt(v);
        } catch (NumberFormatException e) {
            return def;
        }
    }

    private static double parseDoubleEnv(String key, double def) {
        String v = envOrNull(key);
        if (v == null) {
            return def;
        }
        try {
            return Double.parseDouble(v);
        } catch (NumberFormatException e) {
            return def;
        }
    }

    private static void printUsage() {
        System.out.println("Vehicle / signal options:");
        System.out.println("  --signal-ws-port <port>   WebSocket signals (default 9091, 0=off)");
        System.out.println("  --no-signal-ws            Disable WebSocket signal server");
        System.out.println("  --signal-ws-hz <hz>       WebSocket publish rate (default 10)");
        System.out.println("  --vehicle-pose-mode sim|udp|constant");
        System.out.println("  --udp-listen-port <port>  Ingress LE [steering,heading] (udp mode)");
        System.out.println("  --udp-target <host>       Optional egress to Android phone IP");
        System.out.println("  --udp-port <port>         Egress port (default 45454)");
        System.out.println("  --udp-hz <hz>             Egress rate (default 20)");
        System.out.println("  Env: SIGNAL_WS_PORT, VEHICLE_POSE_MODE, UDP_LISTEN_PORT, UDP_TARGET_HOST");
    }
}
