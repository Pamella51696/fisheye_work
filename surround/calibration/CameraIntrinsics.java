package surround.calibration;

public final class CameraIntrinsics {

    public final String model;
    public final int width;
    public final int height;
    public final double fx;
    public final double fy;
    public final double cx;
    public final double cy;
    public final double k1;
    public final double k2;
    public final double k3;
    public final double k4;
    public final double rectBalance;
    public final double rectOutputFovDeg;
    /** &gt;1 widens effective FOV (zoom out) when sampling the fisheye. */
    public final double focalZoomOut;

    public CameraIntrinsics(String model, int width, int height,
                            double fx, double fy, double cx, double cy,
                            double k1, double k2, double k3, double k4,
                            double rectBalance, double rectOutputFovDeg,
                            double focalZoomOut) {
        this.model = model;
        this.width = width;
        this.height = height;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.k1 = k1;
        this.k2 = k2;
        this.k3 = k3;
        this.k4 = k4;
        this.rectBalance = rectBalance;
        this.rectOutputFovDeg = rectOutputFovDeg;
        this.focalZoomOut = focalZoomOut < 0.5 ? 0.5 : focalZoomOut;
    }
}
