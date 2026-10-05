package fisheye;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Resolves per-camera Kannala–Brandt intrinsics from {@code calib/<role>_fisheye.json}
 * or equidistant defaults from image size and FOV.
 */
public final class FisheyeCalibrationStore {

    private FisheyeCalibrationStore() {
    }

    public static KannalaBrandtIntrinsics forCamera(String role, int width, int height,
                                                    double defaultFovDeg,
                                                    double cxFrac, double cyFrac) {
        Path calibDir = Paths.get("calib");
        Path json = calibDir.resolve(role + "_fisheye.json");
        Path props = calibDir.resolve(role + "_fisheye.properties");
        try {
            if (Files.isRegularFile(json)) {
                return KannalaBrandtIntrinsics.load(json, width, height,
                        defaultFovDeg, cxFrac, cyFrac);
            }
            if (Files.isRegularFile(props)) {
                return KannalaBrandtIntrinsics.load(props, width, height,
                        defaultFovDeg, cxFrac, cyFrac);
            }
        } catch (Exception e) {
            System.err.println("Could not load fisheye calib for " + role + ": " + e.getMessage()
                    + " — using FOV defaults.");
        }
        return KannalaBrandtIntrinsics.fromFov(width, height, defaultFovDeg, cxFrac, cyFrac);
    }
}
