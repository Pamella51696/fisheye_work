package surround.calibration;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class RigConfig {

    public final String calibrationMode;
    public final String description;
    public final List<String> cameraOrder;
    public final PanoramaSettings panorama;
    public final List<CameraConfig> cameras;

    public RigConfig(String calibrationMode, String description,
                     List<String> cameraOrder, PanoramaSettings panorama,
                     List<CameraConfig> cameras) {
        this.calibrationMode = calibrationMode;
        this.description = description;
        this.cameraOrder = cameraOrder;
        this.panorama = panorama;
        this.cameras = cameras;
    }

    public CameraConfig camera(String role) {
        for (CameraConfig c : cameras) {
            if (c.role.equalsIgnoreCase(role)) {
                return c;
            }
        }
        throw new IllegalArgumentException("unknown camera role: " + role);
    }

    @SuppressWarnings("unchecked")
    static RigConfig fromJsonMap(Map<String, Object> root) {
        String mode = JsonUtil.asString(root.get("calibration_mode"));
        String desc = root.containsKey("description") ? JsonUtil.asString(root.get("description")) : "";
        List<String> order = new ArrayList<>();
        for (Object o : JsonUtil.asArray(root.get("camera_order"))) {
            order.add(JsonUtil.asString(o));
        }
        Map<String, Object> pano = JsonUtil.asObject(root.get("panorama"));
        boolean ground = pano.containsKey("ground_plane_enabled")
                && JsonUtil.asDouble(pano.get("ground_plane_enabled")) != 0;
        PanoramaSettings panorama = new PanoramaSettings(
                (int) JsonUtil.asDouble(pano.get("panel_width")),
                (int) JsonUtil.asDouble(pano.get("panel_height")),
                JsonUtil.asDouble(pano.get("panel_yaw_deg")),
                JsonUtil.asDouble(pano.get("panel_pitch_deg")),
                JsonUtil.asDouble(pano.get("horizon_fraction")),
                JsonUtil.asDouble(pano.get("max_incidence_deg")),
                (int) JsonUtil.asDouble(pano.get("overlap_px")),
                (int) JsonUtil.asDouble(pano.get("edge_feather_px")),
                ground,
                JsonUtil.asDouble(pano, "camera_height_z", 1.0),
                JsonUtil.asDouble(pano, "ground_plane_z", 0.0),
                JsonUtil.asDouble(pano, "ground_distance_near", 0.35),
                JsonUtil.asDouble(pano, "ground_distance_far", 12.0),
                (int) JsonUtil.asDouble(pano, "ground_blend_rows", 28));
        Map<String, Object> cams = JsonUtil.asObject(root.get("cameras"));
        List<CameraConfig> cameras = new ArrayList<>();
        for (String role : order) {
            Map<String, Object> c = JsonUtil.asObject(cams.get(role));
            CameraIntrinsics intr = new CameraIntrinsics(
                    JsonUtil.asString(c.get("model")),
                    (int) JsonUtil.asDouble(c.get("width")),
                    (int) JsonUtil.asDouble(c.get("height")),
                    JsonUtil.asDouble(c.get("fx")),
                    JsonUtil.asDouble(c.get("fy")),
                    JsonUtil.asDouble(c.get("cx")),
                    JsonUtil.asDouble(c.get("cy")),
                    JsonUtil.asDouble(c.get("k1")),
                    JsonUtil.asDouble(c.get("k2")),
                    JsonUtil.asDouble(c.get("k3")),
                    JsonUtil.asDouble(c.get("k4")),
                    JsonUtil.asDouble(c.get("rect_balance")),
                    JsonUtil.asDouble(c.get("rect_output_fov_deg")),
                    JsonUtil.asDouble(c, "focal_zoom_out", 1.0));
            CameraExtrinsics ext = new CameraExtrinsics(
                    JsonUtil.asDouble(c.get("yaw_deg")),
                    JsonUtil.asDouble(c.get("pitch_deg")),
                    JsonUtil.asDouble(c.get("roll_deg")),
                    JsonUtil.asDouble(c.get("yaw_offset_deg")),
                    JsonUtil.asDouble(c.get("pitch_offset_deg")));
            cameras.add(new CameraConfig(role, intr, ext));
        }
        return new RigConfig(mode, desc, order, panorama, cameras);
    }
}
