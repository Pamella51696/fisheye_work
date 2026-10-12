package com.middleware.panorama.wp5;

/**
 * Idle demo vehicle when no UDP feed is available: stable steering and curb distances
 * (no rapid sine sweeps). Activated by {@link VehicleSignalManager} or -Dwp5.simulate=true.
 * NEVER enable in a real vehicle.
 */
final class Wp5Simulator {
    private final Wp5Service svc;
    Wp5Simulator(Wp5Service svc) { this.svc = svc; }

    void start() {
        Thread t = newWorker();
        t.setDaemon(true);
        t.start();
    }

    /** Same loop as {@link #start()}; interrupt the returned thread to stop. */
    Thread startManaged() {
        Thread t = newWorker();
        t.start();
        return t;
    }

    private Thread newWorker() {
        return new Thread(() -> {
            // Gentle motion only: slow ±25° over 90 s so the viewer stays readable.
            long t0 = System.currentTimeMillis();
            final double rear = 1.8;
            final double curb = 0.85;
            while (!Thread.currentThread().isInterrupted()) {
                double sec = (System.currentTimeMillis() - t0) / 1000.0;
                double steer = 25.0 * Math.sin(2 * Math.PI * sec / 90.0);
                svc.onSteeringInput(steer, "R", 1.5);

                svc.onRangeJson("{\"sensors\":["
                    + s("RC", "REAR_CENTER", rear, "OBSTACLE", "ULTRASONIC", Double.NaN) + ","
                    + s("LR", "LEFT_REAR", curb, "CURB", "ULTRASONIC", 0.12) + ","
                    + s("RR", "RIGHT_REAR", 2.5, "CURB", "ULTRASONIC", Double.NaN) + "]}");
                try { Thread.sleep(250); } catch (InterruptedException e) { return; } // 4 Hz refresh (staleMs)
            }
        }, "wp5-simulator");
    }

    private static String s(String id, String pos, double d, String kind, String src, double h) {
        return "{\"id\":\"" + id + "\",\"pos\":\"" + pos + "\",\"distanceM\":" + Json.num(d)
            + ",\"kind\":\"" + kind + "\",\"source\":\"" + src + "\""
            + (Double.isNaN(h) ? "" : ",\"heightM\":" + Json.num(h)) + "}";
    }
}
