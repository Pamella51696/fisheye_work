package surround.vehicle;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Sends {@code [steeringDeg, headingDeg]} as two little-endian IEEE-754 doubles (16 bytes)
 * over UDP at a fixed rate (~20 Hz by default) — Android legacy receiver contract.
 */
public final class UdpVehiclePosePublisher implements AutoCloseable {

    private static final int PAYLOAD_BYTES = 16;

    private final VehiclePoseSource source;
    private final InetAddress targetHost;
    private final int targetPort;
    private final long periodNanos;
    private final ScheduledExecutorService scheduler =
            Executors.newSingleThreadScheduledExecutor(r -> {
                Thread t = new Thread(r, "udp-vehicle-egress");
                t.setDaemon(true);
                return t;
            });
    private final AtomicBoolean running = new AtomicBoolean(true);
    private DatagramSocket socket;

    public UdpVehiclePosePublisher(
            VehiclePoseSource source,
            String targetHost,
            int targetPort,
            double hz)
            throws UnknownHostException {
        this.source = source;
        this.targetHost = InetAddress.getByName(targetHost);
        this.targetPort = targetPort;
        double rate = hz > 0 ? hz : 20.0;
        this.periodNanos = (long) (1_000_000_000L / rate);
    }

    public synchronized void start() throws SocketException {
        socket = new DatagramSocket();
        scheduler.scheduleAtFixedRate(this::tick, periodNanos, periodNanos, TimeUnit.NANOSECONDS);
        System.out.println("UDP vehicle egress -> " + targetHost.getHostAddress() + ":" + targetPort
                + "  (" + (1_000_000_000.0 / periodNanos) + " Hz, "
                + PAYLOAD_BYTES + " bytes LE [steering, heading])");
    }

    private void tick() {
        if (!running.get()) {
            return;
        }
        DatagramSocket s = socket;
        if (s == null) {
            return;
        }
        VehiclePose pose = source.sample(System.nanoTime());
        ByteBuffer buf = ByteBuffer.allocate(PAYLOAD_BYTES).order(ByteOrder.LITTLE_ENDIAN);
        buf.putDouble(pose.steeringDeg);
        buf.putDouble(pose.headingDeg);
        try {
            DatagramPacket packet = new DatagramPacket(buf.array(), PAYLOAD_BYTES, targetHost, targetPort);
            s.send(packet);
        } catch (IOException e) {
            System.err.println("UDP vehicle egress send failed: " + e.getMessage());
        }
    }

    @Override
    public void close() {
        running.set(false);
        scheduler.shutdownNow();
        DatagramSocket s = socket;
        socket = null;
        if (s != null) {
            s.close();
        }
    }
}
