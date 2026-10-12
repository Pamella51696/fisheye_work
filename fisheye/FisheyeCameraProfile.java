package fisheye;

import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Per-camera frozen lens + small extrinsic trims (strategy B for generated footage).
 */
public final class FisheyeCameraProfile {

    public final FisheyeLensModel lens;
    public final double yawOffsetDeg;
    public final double pitchOffsetDeg;

    public FisheyeCameraProfile(FisheyeLensModel lens,
                                double yawOffsetDeg, double pitchOffsetDeg) {
        this.lens = lens;
        this.yawOffsetDeg = yawOffsetDeg;
        this.pitchOffsetDeg = pitchOffsetDeg;
    }

    public static FisheyeCameraProfile load(Path path, int width, int height,
                                            double defaultFovDeg,
                                            double cxFrac, double cyFrac) throws java.io.IOException {
        KannalaBrandtIntrinsics intrinsics;
        FisheyeProjection projection = FisheyeProjection.EQUIDISTANT;
        double xi = 0;
        double alpha = 0.5;
        double yawOff = 0;
        double pitchOff = 0;

        if (path != null && Files.isRegularFile(path)) {
            String text = Files.readString(path);
            intrinsics = KannalaBrandtIntrinsics.load(path, width, height,
                    defaultFovDeg, cxFrac, cyFrac);
            projection = FisheyeProjection.parse(KannalaBrandtIntrinsics.readString(text, "projection"));
            xi = KannalaBrandtIntrinsics.readNumber(text, "xi", 0);
            alpha = KannalaBrandtIntrinsics.readNumber(text, "alpha", 0.5);
            yawOff = KannalaBrandtIntrinsics.readNumber(text, "yaw_offset_deg", 0);
            pitchOff = KannalaBrandtIntrinsics.readNumber(text, "pitch_offset_deg", 0);
        } else {
            intrinsics = KannalaBrandtIntrinsics.fromFov(width, height, defaultFovDeg, cxFrac, cyFrac);
        }

        FisheyeLensModel lens = new ConfigurableFisheyeLens(intrinsics, projection, xi, alpha);
        return new FisheyeCameraProfile(lens, yawOff, pitchOff);
    }
}
