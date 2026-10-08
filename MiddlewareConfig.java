public final class MiddlewareConfig {
    public final int httpPort;
    public final String udpTargetHost;
    public final int udpPort;
    public final double udpHz;
    public final String poseMode;
    public final double constantSteeringDeg;
    public final double constantHeadingDeg;

    private MiddlewareConfig(
            int httpPort,
            String udpTargetHost,
            int udpPort,
            double udpHz,
            String poseMode,
            double constantSteeringDeg,
            double constantHeadingDeg) {
        this.httpPort = httpPort;
        this.udpTargetHost = udpTargetHost;
        this.udpPort = udpPort;
        this.udpHz = udpHz;
        this.poseMode = poseMode;
        this.constantSteeringDeg = constantSteeringDeg;
        this.constantHeadingDeg = constantHeadingDeg;
    }

    public static MiddlewareConfig fromEnvironmentAndArgs(String[] args, int defaultHttpPort) {
        String udpTarget = envOrNull("UDP_TARGET_HOST");
        int udpPort = parseIntEnv("UDP_PORT", 45454);
        double udpHz = parseDoubleEnv("UDP_HZ", 20.0);
        String poseMode = envOrDefault("POSE_MODE", "sim");
        double steering = parseDoubleEnv("VEHICLE_STEERING_DEG", 0.0);
        double heading = parseDoubleEnv("VEHICLE_HEADING_DEG", 0.0);

        int httpPort = defaultHttpPort;
        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            if ("--udp-target".equals(a) && i + 1 < args.length) {
                udpTarget = args[++i];
            } else if ("--udp-port".equals(a) && i + 1 < args.length) {
                udpPort = Integer.parseInt(args[++i]);
            } else if ("--udp-hz".equals(a) && i + 1 < args.length) {
                udpHz = Double.parseDouble(args[++i]);
            } else if ("--pose-mode".equals(a) && i + 1 < args.length) {
                poseMode = args[++i];
            } else if ("--steering-deg".equals(a) && i + 1 < args.length) {
                steering = Double.parseDouble(args[++i]);
            } else if ("--heading-deg".equals(a) && i + 1 < args.length) {
                heading = Double.parseDouble(args[++i]);
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

        return new MiddlewareConfig(httpPort, udpTarget, udpPort, udpHz, poseMode, steering, heading);
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
            System.err.println("Invalid " + key + "=" + v + ", using " + def);
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
            System.err.println("Invalid " + key + "=" + v + ", using " + def);
            return def;
        }
    }

    static void printUsage() {
        System.out.println("Options:");
        System.out.println("  <port>                 HTTP panorama port (default " + 9090 + ")");
        System.out.println("  --udp-target <host>    Phone IP for pose UDP (or env UDP_TARGET_HOST)");
        System.out.println("  --udp-port <port>      UDP listen port on phone (default 45454)");
        System.out.println("  --udp-hz <rate>        Pose send rate (default 20)");
        System.out.println("  --pose-mode sim|constant|live  (live: feed via MiddlewarePoseBus.update)");
        System.out.println("  --steering-deg <deg>   For constant mode (0=forward, 90=right)");
        System.out.println("  --heading-deg <deg>    0=North, 90=East; sim mode uses as offset");
    }
}
