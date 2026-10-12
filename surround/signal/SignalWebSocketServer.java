package surround.signal;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Dedicated signal channel on its own port (default 9091). Pushes the same JSON as
 * {@code GET /api/signals} at a fixed rate with {@link SignalPublisher} deadband applied.
 */
public final class SignalWebSocketServer implements AutoCloseable {

    private static final String PATH = "/signals";

    private final int port;
    private final SignalPublisher publisher;
    private final double hz;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final CopyOnWriteArrayList<Socket> clients = new CopyOnWriteArrayList<>();
    private ServerSocket serverSocket;
    private Thread acceptThread;
    private ScheduledExecutorService broadcaster;

    public SignalWebSocketServer(int port, SignalPublisher publisher, double hz) {
        this.port = port;
        this.publisher = publisher;
        this.hz = hz > 0 ? hz : 10.0;
    }

    public void start() throws IOException {
        if (port <= 0) {
            return;
        }
        if (!running.compareAndSet(false, true)) {
            return;
        }
        serverSocket = new ServerSocket();
        serverSocket.bind(new InetSocketAddress(port));
        acceptThread = new Thread(this::acceptLoop, "signal-ws-accept");
        acceptThread.setDaemon(true);
        acceptThread.start();

        long periodMs = Math.max(1, (long) (1000.0 / hz));
        broadcaster = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "signal-ws-broadcast");
            t.setDaemon(true);
            return t;
        });
        broadcaster.scheduleAtFixedRate(this::broadcast, periodMs, periodMs, TimeUnit.MILLISECONDS);
        System.out.println("Signal WebSocket ws://localhost:" + port + PATH
                + "  (~" + hz + " Hz JSON)");
    }

    private void acceptLoop() {
        while (running.get()) {
            try {
                Socket socket = serverSocket.accept();
                Thread t = new Thread(() -> handleClient(socket), "signal-ws-client");
                t.setDaemon(true);
                t.start();
            } catch (IOException e) {
                if (running.get()) {
                    System.err.println("Signal WebSocket accept: " + e.getMessage());
                }
                break;
            }
        }
    }

    private void handleClient(Socket socket) {
        try (Socket s = socket) {
            s.setTcpNoDelay(true);
            InputStream in = s.getInputStream();
            OutputStream out = s.getOutputStream();
            if (!WebSocketHandshake.perform(in, out, PATH)) {
                return;
            }
            clients.add(s);
            drainClientFrames(in);
        } catch (IOException ignored) {
        } finally {
            clients.remove(socket);
        }
    }

    /** Read client frames until disconnect (we do not process client messages). */
    private void drainClientFrames(InputStream in) throws IOException {
        while (running.get()) {
            int b = in.read();
            if (b < 0) {
                return;
            }
            if ((b & 0x0F) == 0x8) {
                return;
            }
            int lenByte = in.read();
            if (lenByte < 0) {
                return;
            }
            boolean masked = (lenByte & 0x80) != 0;
            int len = lenByte & 0x7F;
            if (len == 126) {
                int hi = in.read();
                int lo = in.read();
                if (hi < 0 || lo < 0) {
                    return;
                }
                len = (hi << 8) | lo;
            } else if (len == 127) {
                return;
            }
            if (masked) {
                byte[] masking = new byte[4];
                if (in.read(masking) != 4) {
                    return;
                }
            }
            long skipped = 0;
            while (skipped < len) {
                long n = in.skip(len - skipped);
                if (n <= 0) {
                    if (in.read() < 0) {
                        return;
                    }
                    skipped++;
                } else {
                    skipped += n;
                }
            }
        }
    }

    private void broadcast() {
        if (clients.isEmpty()) {
            return;
        }
        String json = publisher.toJson();
        for (Socket s : clients) {
            try {
                OutputStream out = s.getOutputStream();
                WebSocketHandshake.writeTextFrame(out, json);
            } catch (IOException e) {
                clients.remove(s);
                try {
                    s.close();
                } catch (IOException ignored) {
                }
            }
        }
    }

    @Override
    public void close() {
        running.set(false);
        if (broadcaster != null) {
            broadcaster.shutdownNow();
        }
        for (Socket s : clients) {
            try {
                s.close();
            } catch (IOException ignored) {
            }
        }
        clients.clear();
        ServerSocket ss = serverSocket;
        serverSocket = null;
        if (ss != null) {
            try {
                ss.close();
            } catch (IOException ignored) {
            }
        }
        if (acceptThread != null) {
            acceptThread.interrupt();
        }
    }
}
