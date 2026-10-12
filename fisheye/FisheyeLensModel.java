package fisheye;

/**
 * Maps a unit camera ray (OpenCV: X right, Y down, Z forward) to distorted image pixels.
 */
public interface FisheyeLensModel {

    void rayToPixel(double x, double y, double z, double[] outUv);

    KannalaBrandtIntrinsics intrinsics();

    FisheyeProjection projection();
}
