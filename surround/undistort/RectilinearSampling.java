package surround.undistort;

/** Pinhole intrinsics of a {@link RectilinearMapper} output image. */
public final class RectilinearSampling {

    public final double fx;
    public final double fy;
    public final double cx;
    public final double cy;

    public RectilinearSampling(double fx, double fy, double cx, double cy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }
}
