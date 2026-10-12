package surround.signal;

import java.util.Locale;

import output.CurbZoneType;
import surround.analytics.CurbDetectionService;
import surround.analytics.CurbState;
import surround.vehicle.UdpVehicleSignalService;
import surround.vehicle.VehicleSignal;

/** Unified Android-facing JSON (video remains on separate MJPEG endpoints). */
public final class SignalPublisher {

    private final CurbDetectionService curbService;
    private final UdpVehicleSignalService vehicleService;
    private final double steeringDeadbandDeg;

    public SignalPublisher(CurbDetectionService curbService,
                           UdpVehicleSignalService vehicleService,
                           double steeringDeadbandDeg) {
        this.curbService = curbService;
        this.vehicleService = vehicleService;
        this.steeringDeadbandDeg = steeringDeadbandDeg;
    }

    private double lastSentSteering = Double.NaN;

    public synchronized String toJson() {
        long ts = System.currentTimeMillis();
        CurbState curb = curbService != null ? curbService.currentState() : CurbState.unavailable();
        VehicleSignal vehicle = vehicleService.getCurrentSignal();

        String curbBlock;
        if (curb.status == CurbState.Status.UNAVAILABLE) {
            curbBlock = "{\"status\":\"UNAVAILABLE\"}";
        } else {
            curbBlock = String.format(Locale.ROOT,
                    "{\"detected\":%s,\"distanceMeters\":%s,\"zone\":\"%s\",\"confidence\":%.3f,\"camera\":\"%s\"}",
                    curb.detected,
                    num(curb.distanceMeters),
                    curb.zone == null ? CurbZoneType.UNKNOWN.name() : curb.zone.name(),
                    curb.confidence,
                    escape(curb.camera));
        }

        double steering = vehicle.steeringAngleDeg;
        if (!Double.isNaN(lastSentSteering)
                && Math.abs(steering - lastSentSteering) < steeringDeadbandDeg) {
            steering = lastSentSteering;
        } else {
            lastSentSteering = steering;
        }

        String vehicleBlock;
        if (vehicle.status == VehicleSignal.Status.UNAVAILABLE) {
            vehicleBlock = "{\"status\":\"UNAVAILABLE\"}";
        } else {
            vehicleBlock = String.format(Locale.ROOT,
                    "{\"steeringAngleDeg\":%.2f,\"yawAngleDeg\":%.2f,\"velocityMps\":%.2f}",
                    steering, vehicle.yawAngleDeg, vehicle.velocityMps);
        }

        return String.format(Locale.ROOT,
                "{\"timestamp\":%d,\"curb\":%s,\"vehicle\":%s}", ts, curbBlock, vehicleBlock);
    }

    private static String num(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "null";
        }
        return String.format(Locale.ROOT, "%.3f", v);
    }

    private static String escape(String s) {
        if (s == null) {
            return "";
        }
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
