import geometry.CoordinateTransform;
import output.CurbZoneType;

/**
 * Lightweight unit checks (no OpenCV). Run after compile:
 *   java -cp out tests.CurbGeometryTest
 */
public class CurbGeometryTest {

    public static void main(String[] args) {
        testZones();
        testGroundRange();
        System.out.println("CurbGeometryTest OK");
    }

    static void testZones() {
        assert CurbZoneType.fromDistance(0.3, 0.5, 1.0) == CurbZoneType.RED;
        assert CurbZoneType.fromDistance(0.7, 0.5, 1.0) == CurbZoneType.YELLOW;
        assert CurbZoneType.fromDistance(1.2, 0.5, 1.0) == CurbZoneType.GREEN;
    }

    static void testGroundRange() {
        double[] R = CoordinateTransform.rotationCameraToVehicle(0, -12, 0);
        double fx = 400;
        double fy = 400;
        double cx = 320;
        double cy = 240;
        double d = CoordinateTransform.groundRangeMeters(
                cx, cy + 80, fx, fy, cx, cy, R, 1.0, 0.0);
        if (Double.isNaN(d) || d <= 0) {
            throw new AssertionError("expected positive ground range, got " + d);
        }
    }
}
