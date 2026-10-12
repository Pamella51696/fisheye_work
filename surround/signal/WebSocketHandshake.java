package surround.signal;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Base64;

/** Minimal RFC 6455 server handshake (no external WebSocket library). */
final class WebSocketHandshake {

    private static final String GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

    private WebSocketHandshake() {
    }

    static boolean perform(InputStream in, OutputStream out, String expectedPath) throws IOException {
        String request = readHttpHeaders(in);
        if (request.isEmpty()) {
            return false;
        }
        String path = null;
        String key = null;
        for (String line : request.split("\r\n")) {
            if (line.startsWith("GET ")) {
                int end = line.indexOf(' ', 4);
                path = end > 4 ? line.substring(4, end) : "/";
            } else if (line.regionMatches(true, 0, "Sec-WebSocket-Key:", 0, 18)) {
                key = line.substring(18).trim();
            }
        }
        if (key == null) {
            writeHttp(out, 400, "Bad Request", "Missing Sec-WebSocket-Key");
            return false;
        }
        if (expectedPath != null && path != null && !path.equals(expectedPath) && !path.startsWith(expectedPath + "?")) {
            writeHttp(out, 404, "Not Found", "Expected path " + expectedPath);
            return false;
        }
        String accept = acceptKey(key);
        out.write(("HTTP/1.1 101 Switching Protocols\r\n"
                + "Upgrade: websocket\r\n"
                + "Connection: Upgrade\r\n"
                + "Sec-WebSocket-Accept: " + accept + "\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
        out.flush();
        return true;
    }

    private static String readHttpHeaders(InputStream in) throws IOException {
        ByteArrayOutputStream buf = new ByteArrayOutputStream();
        int prev = 0;
        int prev2 = 0;
        int prev3 = 0;
        while (true) {
            int b = in.read();
            if (b < 0) {
                break;
            }
            buf.write(b);
            if (prev3 == '\r' && prev2 == '\n' && prev == '\r' && b == '\n') {
                break;
            }
            prev3 = prev2;
            prev2 = prev;
            prev = b;
        }
        return buf.toString(StandardCharsets.US_ASCII);
    }

    private static void writeHttp(OutputStream out, int code, String status, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        out.write(("HTTP/1.1 " + code + " " + status + "\r\n"
                + "Content-Length: " + bytes.length + "\r\n"
                + "Connection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
        out.flush();
    }

    private static String acceptKey(String key) {
        try {
            MessageDigest sha1 = MessageDigest.getInstance("SHA-1");
            sha1.update((key + GUID).getBytes(StandardCharsets.US_ASCII));
            return Base64.getEncoder().encodeToString(sha1.digest());
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }

    static void writeTextFrame(OutputStream out, String text) throws IOException {
        byte[] payload = text.getBytes(StandardCharsets.UTF_8);
        out.write(0x81);
        if (payload.length <= 125) {
            out.write(payload.length);
        } else if (payload.length <= 65535) {
            out.write(126);
            out.write((payload.length >> 8) & 0xFF);
            out.write(payload.length & 0xFF);
        } else {
            throw new IOException("WebSocket text frame too large");
        }
        out.write(payload);
        out.flush();
    }
}
