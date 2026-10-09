package surround.vehicle;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Listens for Android / vehicle bus packets: 16 bytes little-endian
 * {@code [steeringDeg, headingDeg]} (same contract as outbound UDP publisher).
 */
public final class UdpInboundPoseListener implements VehicleSignalProvider, AutoCloseable {

    private static final int PAYLOAD_BYTES = 16;

    private final int listenPort;
    private final LatestVehiclePoseSource pose = new LatestVehiclePoseSource(0.0, 0.0);
    private final AtomicBoolean running = new AtomicBoolean(false);
    private DatagramSocket socket;
    private Thread thread;
    private volatile long lastPacketMs;
    private volatile boolean everReceived;

    public UdpInboundPoseListener(int listenPort) {
        this.listenPort = listenPort;
    }

    public void start() throws SocketException {
        if (!running.compareAndSet(false, true)) {
            return;
        }
        socket = new DatagramSocket(listenPort);
        thread = new Thread(this::loop, "udp-vehicle-ingress");
        thread.setDaemon(true);
        thread.start();
        System.out.println("UDP vehicle ingress listening on port " + listenPort
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
                double steering = bb.getDouble();
                double heading = bb.getDouble();
                pose.update(steering, heading);
                lastPacketMs = System.currentTimeMillis();
                everReceived = true;
            } catch (IOException e) {
                if (running.get()) {
                    System.err.println("UDP vehicle ingress: " + e.getMessage());
                }
                break;
            }
        }
    }

    @Override
    public VehicleSignal getCurrentSignal() {
        if (!everReceived) {
            return VehicleSignal.unavailable();
        }
        long age = System.currentTimeMillis() - lastPacketMs;
        if (age > 2000) {
            return VehicleSignal.unavailable();
        }
        VehiclePose p = pose.sample(0);
        return new VehicleSignal(lastPacketMs, VehicleSignal.Status.OK,
                p.steeringDeg, p.headingDeg, 0.0);
    }

    public LatestVehiclePoseSource poseSource() {
        return pose;
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
