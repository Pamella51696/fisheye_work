package surround.calibration;

public final class CameraConfig {

    public final String role;
    public final CameraIntrinsics intrinsics;
    public final CameraExtrinsics extrinsics;

    public CameraConfig(String role, CameraIntrinsics intrinsics, CameraExtrinsics extrinsics) {
        this.role = role;
        this.intrinsics = intrinsics;
        this.extrinsics = extrinsics;
    }
}
