package com.middleware.panorama.wp5;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Listens for 16-byte little-endian {@code [steeringDeg, headingDeg]} UDP packets
 * (same contract as the udp-vehicle-pose middleware branch).
 */
final class UdpVehicleSignalReceiver implements AutoCloseable {

    private static final int PAYLOAD_BYTES = 16;

    private final int listenPort;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private volatile double lastSteeringDeg;
    private volatile double lastHeadingDeg;
    private volatile long lastPacketMs;
    private volatile boolean everReceived;
    private DatagramSocket socket;
    private Thread thread;

    UdpVehicleSignalReceiver(int listenPort) {
        this.listenPort = listenPort;
    }

    void start() throws SocketException {
        if (!running.compareAndSet(false, true)) {
            return;
        }
        socket = new DatagramSocket(listenPort);
        thread = new Thread(this::loop, "wp5-udp-ingress");
        thread.setDaemon(true);
        thread.start();
        System.out.println("[WP5] UDP ingress on port " + listenPort
                + " (16-byte LE [steeringDeg, headingDeg])");
    }

    private void loop() {
        byte[] buf = new byte[PAYLOAD_BYTES];
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        while (running.get()) {
            try {
                socket.receive(packet);
                if (packet.getLength() < PAYLOAD_BYTES) {
                    continue;
                }
                ByteBuffer bb = ByteBuffer.wrap(buf, 0, PAYLOAD_BYTES).order(ByteOrder.LITTLE_ENDIAN);
                lastSteeringDeg = bb.getDouble();
                lastHeadingDeg = bb.getDouble();
                lastPacketMs = System.currentTimeMillis();
                everReceived = true;
            } catch (IOException e) {
                if (running.get()) {
                    System.err.println("[WP5] UDP ingress: " + e.getMessage());
                }
                break;
            }
        }
    }

    boolean isFresh(long timeoutMs) {
        if (!everReceived) {
            return false;
        }
        return System.currentTimeMillis() - lastPacketMs <= timeoutMs;
    }

    long lastPacketMs() {
        return lastPacketMs;
    }

    double steeringDeg() {
        return lastSteeringDeg;
    }

    double headingDeg() {
        return lastHeadingDeg;
    }

    @Override
    public void close() {
        running.set(false);
        DatagramSocket s = socket;
        socket = null;
        if (s != null) {
            s.close();
        }
        if (thread != null) {
            thread.interrupt();
        }
    }
}
