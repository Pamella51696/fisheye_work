package com.middleware.panorama.wp5;

/** Which vehicle steering feed is active (see {@link VehicleSignalManager}). */
public enum SignalSource {
    AUTO,
    UDP,
    SIMULATOR;

    static SignalSource parse(String raw) {
        if (raw == null || raw.isBlank()) {
            return null;
        }
        switch (raw.trim().toUpperCase()) {
            case "AUTO":
                return AUTO;
            case "UDP":
            case "LIVE":
                return UDP;
            case "SIMULATOR":
            case "SIM":
            case "SIMULATE":
                return SIMULATOR;
            default:
                System.err.println("[WP5] unknown wp5.signalSource='" + raw + "', using AUTO");
                return AUTO;
        }
    }
}
