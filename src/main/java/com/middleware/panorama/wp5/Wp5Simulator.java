package com.middleware.panorama.wp5;

/**
 * Fake vehicle for development: sweeps the steering wheel left/right in reverse gear
 * and moves obstacles closer/further so every zone colour appears.
 * Enable with -Dwp5.simulate=true. NEVER enable in a real vehicle.
 */
final class Wp5Simulator {
    private final Wp5Service svc;
    Wp5Simulator(Wp5Service svc) { this.svc = svc; }

    void start() {
        Thread t = new Thread(() -> {
            long t0 = System.currentTimeMillis();
            while (!Thread.currentThread().isInterrupted()) {
                double sec = (System.currentTimeMillis() - t0) / 1000.0;
                double steer = 450.0 * Math.sin(2 * Math.PI * sec / 12.0);          // +-450 deg, 12 s period
                svc.onSteeringInput(steer, "R", 1.5);

                double rear = 1.35 + 1.15 * Math.sin(2 * Math.PI * sec / 8.0);      // 0.20 .. 2.50 m
                double curb = 0.70 + 0.45 * Math.sin(2 * Math.PI * sec / 6.0 + 1);  // 0.25 .. 1.15 m
                double right = 2.5;                                                  // clear
                svc.onRangeJson("{\"sensors\":["
                    + s("RC", "REAR_CENTER", rear, "OBSTACLE", "ULTRASONIC", Double.NaN) + ","
                    + s("LR", "LEFT_REAR", curb, "CURB", "ULTRASONIC", 0.12) + ","
                    + s("RR", "RIGHT_REAR", right, "CURB", "ULTRASONIC", Double.NaN) + "]}");
                try { Thread.sleep(50); } catch (InterruptedException e) { return; } // 20 Hz
            }
        }, "wp5-simulator");
        t.setDaemon(true);
        t.start();
    }

    private static String s(String id, String pos, double d, String kind, String src, double h) {
        return "{\"id\":\"" + id + "\",\"pos\":\"" + pos + "\",\"distanceM\":" + Json.num(d)
            + ",\"kind\":\"" + kind + "\",\"source\":\"" + src + "\""
            + (Double.isNaN(h) ? "" : ",\"heightM\":" + Json.num(h)) + "}";
    }
}
