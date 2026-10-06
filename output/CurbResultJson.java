package output;

import java.util.List;

/** JSON serialization for Android / IPC consumers. */
public final class CurbResultJson {

    private CurbResultJson() {
    }

    public static String fusedToJson(FusedCurbResult fused) {
        StringBuilder sb = new StringBuilder(2048);
        sb.append('{');
        field(sb, "timestamp", fused.timestampMs);
        sb.append(",\"curbDetected\":").append(fused.curbDetected);
        sb.append(",\"confidence\":").append(round3(fused.confidence));
        sb.append(",\"valid\":").append(fused.valid);
        sb.append(",\"vehicleDistance\":{");
        sb.append("\"min\":").append(num(fused.distanceMinMeters));
        sb.append(",\"max\":").append(num(fused.distanceMaxMeters));
        sb.append(",\"unit\":\"meters\"}");
        sb.append(",\"curb\":{");
        sb.append("\"coordinateSystem\":\"VEHICLE\",");
        sb.append("\"points\":").append(vehiclePoints(fused.curbPointsVehicle));
        sb.append('}');
        sb.append(",\"zones\":").append(zonesJson(fused.zones));
        sb.append(",\"cameras\":").append(cameraArray(fused.perCamera));
        sb.append('}');
        return sb.toString();
    }

    public static String cameraToJson(CurbDetectionResult r) {
        StringBuilder sb = new StringBuilder(1024);
        sb.append('{');
        field(sb, "timestamp", r.timestampMs);
        sb.append(",\"cameraId\":\"").append(escape(r.cameraId)).append('"');
        sb.append(",\"curbDetected\":").append(r.curbDetected);
        sb.append(",\"confidence\":").append(round3(r.confidence));
        sb.append(",\"trackingStatus\":\"").append(r.trackingStatus.name()).append('"');
        sb.append(",\"valid\":").append(r.valid);
        sb.append(",\"curb\":{");
        sb.append("\"coordinateSystem\":\"IMAGE\",");
        sb.append("\"points\":").append(imagePoints(r.curbPoints));
        sb.append('}');
        sb.append(",\"distanceMeters\":{");
        sb.append("\"min\":").append(num(r.distanceMinMeters));
        sb.append(",\"max\":").append(num(r.distanceMaxMeters));
        sb.append('}');
        sb.append(",\"zones\":").append(zonesJson(r.zones));
        sb.append('}');
        return sb.toString();
    }

    private static String cameraArray(List<CurbDetectionResult> list) {
        StringBuilder sb = new StringBuilder();
        sb.append('[');
        for (int i = 0; i < list.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append(cameraToJson(list.get(i)));
        }
        sb.append(']');
        return sb.toString();
    }

    private static String zonesJson(List<CurbZoneSegment> zones) {
        StringBuilder sb = new StringBuilder();
        sb.append('[');
        for (int i = 0; i < zones.size(); i++) {
            CurbZoneSegment z = zones.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append('{');
            sb.append("\"type\":\"").append(z.type.name()).append('"');
            sb.append(",\"distanceMin\":").append(num(z.distanceMin));
            sb.append(",\"distanceMax\":").append(num(z.distanceMax));
            sb.append(",\"points\":").append(imagePoints(z.points));
            sb.append('}');
        }
        sb.append(']');
        return sb.toString();
    }

    private static String imagePoints(List<ImagePoint> pts) {
        StringBuilder sb = new StringBuilder();
        sb.append('[');
        for (int i = 0; i < pts.size(); i++) {
            ImagePoint p = pts.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"x\":").append(round1(p.x)).append(",\"y\":").append(round1(p.y)).append('}');
        }
        sb.append(']');
        return sb.toString();
    }

    private static String vehiclePoints(List<VehiclePoint> pts) {
        StringBuilder sb = new StringBuilder();
        sb.append('[');
        for (int i = 0; i < pts.size(); i++) {
            VehiclePoint p = pts.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"x\":").append(round3(p.x));
            sb.append(",\"y\":").append(round3(p.y));
            sb.append(",\"z\":").append(round3(p.z)).append('}');
        }
        sb.append(']');
        return sb.toString();
    }

    private static void field(StringBuilder sb, String key, long value) {
        sb.append('"').append(key).append("\":").append(value);
    }

    private static String escape(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    private static String num(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "null";
        }
        return String.format(java.util.Locale.ROOT, "%.3f", v);
    }

    private static double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }

    private static double round3(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }
}
