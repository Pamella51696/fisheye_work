package com.middleware.panorama.wp5;

import java.net.SocketException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Selects between {@link UdpVehicleSignalReceiver} and {@link Wp5Simulator} without
 * changing {@link SteeringModel} or {@link SignalHub}. Curb/video paths are untouched.
 */
public final class VehicleSignalManager implements AutoCloseable {

    private enum Active {
        NONE,
        UDP,
        SIMULATOR
    }

    private final Wp5Config cfg;
    private final Wp5Service service;
    private final UdpVehicleSignalReceiver udp;
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "wp5-signal-manager");
        t.setDaemon(true);
        return t;
    });

    private final long startedMs = System.currentTimeMillis();
    private final AtomicReference<Active> active = new AtomicReference<>(Active.NONE);
    private Wp5Simulator simulator;
    private volatile Thread simulatorThread;

    public VehicleSignalManager(Wp5Config cfg, Wp5Service service) {
        this.cfg = cfg;
        this.service = service;
        this.udp = new UdpVehicleSignalReceiver(cfg.udpListenPort);
    }

    public void start() {
        try {
            udp.start();
        } catch (SocketException e) {
            System.err.println("[WP5] UDP listen failed: " + e.getMessage());
            if (cfg.signalSource == SignalSource.UDP) {
                System.err.println("[WP5] signalSource=UDP but socket unavailable — no steering input");
            }
        }

        switch (cfg.signalSource) {
            case SIMULATOR:
                activateSimulator("SIMULATOR");
                break;
            case UDP:
                active.set(Active.UDP);
                System.out.println("[WP5] signalSource=UDP (simulator off; stale if no packets)");
                break;
            case AUTO:
            default:
                System.out.println("[WP5] signalSource=AUTO (UDP preferred, simulator after "
                        + cfg.autoStartupTimeoutMs + " ms if idle)");
                break;
        }

        scheduler.scheduleAtFixedRate(this::tick, 50, 50, TimeUnit.MILLISECONDS);
    }

    /** HTTP gateway may still forward steering when UDP is not the exclusive active source. */
    boolean acceptHttpIngest() {
        SignalSource mode = cfg.signalSource;
        if (mode == SignalSource.SIMULATOR) {
            return false;
        }
        if (mode == SignalSource.UDP && active.get() == Active.UDP && udp.isFresh(cfg.udpTimeoutMs)) {
            return false;
        }
        return true;
    }

    void onHttpSteering(double steeringWheelDeg, String gear, double speedKph) {
        if (!acceptHttpIngest()) {
            return;
        }
        service.onSteeringInput(steeringWheelDeg, gear, speedKph);
    }

    private void tick() {
        try {
            switch (cfg.signalSource) {
                case SIMULATOR:
                    return;
                case UDP:
                    tickUdpOnly();
                    return;
                case AUTO:
                default:
                    tickAuto();
            }
        } catch (Throwable t) {
            System.err.println("[WP5] signal manager: " + t.getMessage());
        }
    }

    private void tickUdpOnly() {
        if (udp.isFresh(cfg.udpTimeoutMs)) {
            active.set(Active.UDP);
            forwardUdp();
        }
    }

    private void tickAuto() {
        boolean fresh = udp.isFresh(cfg.udpTimeoutMs);
        Active now = active.get();

        if (fresh) {
            if (now == Active.SIMULATOR) {
                stopSimulator("UDP resumed");
            }
            active.set(Active.UDP);
            forwardUdp();
            return;
        }

        if (now == Active.UDP) {
            stopSimulator(null);
            activateSimulator("UDP timeout");
            return;
        }

        if (now == Active.NONE) {
            long elapsed = System.currentTimeMillis() - startedMs;
            if (elapsed >= cfg.autoStartupTimeoutMs) {
                activateSimulator("AUTO startup timeout");
            }
        }
    }

    private void forwardUdp() {
        service.onSteeringInput(udp.steeringDeg(), "D", 0.0);
    }

    private void activateSimulator(String reason) {
        if (active.get() == Active.SIMULATOR && simulatorThread != null && simulatorThread.isAlive()) {
            return;
        }
        stopSimulator(null);
        simulator = new Wp5Simulator(service);
        simulatorThread = simulator.startManaged();
        active.set(Active.SIMULATOR);
        System.out.println("[WP5] simulator active (" + reason + ")");
    }

    private void stopSimulator(String reason) {
        Thread t = simulatorThread;
        simulatorThread = null;
        if (t != null) {
            t.interrupt();
        }
        if (reason != null) {
            System.out.println("[WP5] simulator stopped (" + reason + ")");
        }
        if (active.get() == Active.SIMULATOR) {
            active.set(Active.NONE);
        }
    }

    public String statusJson() {
        Active a = active.get();
        boolean udpFresh = udp.isFresh(cfg.udpTimeoutMs);
        return "{\"signalSource\":\"" + cfg.signalSource.name()
                + "\",\"active\":\"" + a.name()
                + "\",\"udpFresh\":" + udpFresh
                + ",\"udpListenPort\":" + cfg.udpListenPort + "}";
    }

    @Override
    public void close() {
        scheduler.shutdownNow();
        stopSimulator("shutdown");
        udp.close();
    }
}
