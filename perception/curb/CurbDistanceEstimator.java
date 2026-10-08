package perception.curb;

import configuration.CurbConfig;
import geometry.CoordinateTransform;
import output.ImagePoint;
import output.VehiclePoint;
import surround.calibration.CameraConfig;
import surround.undistort.RectilinearMapper;
import surround.undistort.RectilinearSampling;

import java.util.ArrayList;
import java.util.List;

public final class CurbDistanceEstimator {

    private final CurbConfig config;
    private final double[] R;

    public CurbDistanceEstimator(CurbConfig config, CameraConfig camera) {
        this.config = config;
        this.R = CoordinateTransform.rotationCameraToVehicle(
                camera.extrinsics.effectiveYawDeg(),
                camera.extrinsics.effectivePitchDeg(),
                camera.extrinsics.rollDeg);
    }

    public double[] distancesMeters(List<ImagePoint> imagePoints,
                                    RectilinearSampling sampling) {
        double[] out = new double[imagePoints.size()];
        for (int i = 0; i < imagePoints.size(); i++) {
            ImagePoint p = imagePoints.get(i);
            double d = CoordinateTransform.groundRangeMeters(
                    p.x, p.y,
                    sampling.fx, sampling.fy, sampling.cx, sampling.cy,
                    R, config.cameraHeightZ, config.groundPlaneZ);
            out[i] = d;
        }
        return out;
    }

    public List<VehiclePoint> toVehicleGround(List<ImagePoint> imagePoints,
                                              RectilinearSampling sampling) {
        List<VehiclePoint> pts = new ArrayList<>();
        double[] xyz = new double[3];
        for (ImagePoint p : imagePoints) {
            CoordinateTransform.groundPointMeters(
                    p.x, p.y,
                    sampling.fx, sampling.fy, sampling.cx, sampling.cy,
                    R, config.cameraHeightZ, config.groundPlaneZ, xyz);
            if (!Double.isNaN(xyz[0])) {
                pts.add(new VehiclePoint(xyz[0], xyz[1], xyz[2]));
            }
        }
        return pts;
    }

    public static RectilinearSampling samplingFor(CameraConfig cam, RectilinearMapper mapper, int w, int h) {
        return mapper.samplingForSize(w, h);
    }
}
