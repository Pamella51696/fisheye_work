package perception.curb;

import configuration.CurbConfig;
import output.CurbDetectionResult;
import output.CurbZoneSegment;
import output.FusedCurbResult;
import output.ImagePoint;
import output.TrackingStatus;
import output.VehiclePoint;
import surround.calibration.CameraConfig;
import surround.calibration.RigConfig;
import surround.runtime.SurroundPipeline;
import surround.undistort.RectilinearMapper;
import surround.undistort.RectilinearSampling;

import org.opencv.core.Mat;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Curb perception layer on top of fisheye middleware (rectilinear feeds + rig geometry).
 */
public final class CurbPerceptionPipeline {

    private final CurbConfig curbConfig;
    private final SurroundPipeline surround;
    private final CurbDetector detector;
    private final CurbFusion fusion;
    private final List<CameraCurbContext> cameras;

    public CurbPerceptionPipeline(CurbConfig curbConfig, SurroundPipeline surround) {
        this.curbConfig = curbConfig;
        this.surround = surround;
        this.detector = new ClassicalCurbDetector();
        this.fusion = new CurbFusion(curbConfig);
        this.cameras = new ArrayList<>();
        RigConfig rig = surround.rig();
        for (CameraConfig cam : rig.cameras) {
            RectilinearMapper mapper = new RectilinearMapper(
                    cam.intrinsics,
                    cam.intrinsics.rectBalance,
                    cam.intrinsics.rectOutputFovDeg);
            cameras.add(new CameraCurbContext(cam, mapper,
                    new CurbTracker(curbConfig),
                    new CurbDistanceEstimator(curbConfig, cam),
                    new CurbZoneClassifier(curbConfig)));
        }
    }

    public static CurbPerceptionPipeline load(Path rigJson, Path curbJson) throws IOException {
        return new CurbPerceptionPipeline(
                CurbConfig.load(curbJson),
                SurroundPipeline.load(rigJson));
    }

    public FusedCurbResult process(Mat[] fisheyeFrames, long timestampMs) {
        List<CurbDetectionResult> perCamera = new ArrayList<>();
        List<List<VehiclePoint>> vehiclePolylines = new ArrayList<>();
        Mat rect = new Mat();

        for (int i = 0; i < cameras.size(); i++) {
            CameraCurbContext ctx = cameras.get(i);
            surround.rectify(i, fisheyeFrames[i], rect);
            perCamera.add(ctx.process(rect, timestampMs));
            vehiclePolylines.add(ctx.lastVehiclePoints);
        }
        rect.release();
        return fusion.fuse(timestampMs, perCamera, vehiclePolylines);
    }

    public CurbConfig config() {
        return curbConfig;
    }

    private final class CameraCurbContext {
        private final CameraConfig camera;
        private final RectilinearMapper mapper;
        private final CurbTracker tracker;
        private final CurbDistanceEstimator distanceEstimator;
        private final CurbZoneClassifier zoneClassifier;
        private List<VehiclePoint> lastVehiclePoints = List.of();

        CameraCurbContext(CameraConfig camera,
                          RectilinearMapper mapper,
                          CurbTracker tracker,
                          CurbDistanceEstimator distanceEstimator,
                          CurbZoneClassifier zoneClassifier) {
            this.camera = camera;
            this.mapper = mapper;
            this.tracker = tracker;
            this.distanceEstimator = distanceEstimator;
            this.zoneClassifier = zoneClassifier;
        }

        CurbDetectionResult process(Mat rectilinear, long timestampMs) {
            double[] detConf = new double[1];
            List<ImagePoint> measured = detector.detect(rectilinear, camera.role, detConf);
            List<ImagePoint> tracked = tracker.update(measured, detConf[0], curbConfig.minConfidence);

            if (tracked.isEmpty()) {
                lastVehiclePoints = List.of();
                return CurbDetectionResult.noDetection(timestampMs, camera.role);
            }

            RectilinearSampling sampling = mapper.samplingForSize(rectilinear.cols(), rectilinear.rows());
            double[] distances = distanceEstimator.distancesMeters(tracked, sampling);
            lastVehiclePoints = distanceEstimator.toVehicleGround(tracked, sampling);

            double min = Double.POSITIVE_INFINITY;
            double max = Double.NEGATIVE_INFINITY;
            int validCount = 0;
            for (double d : distances) {
                if (Double.isNaN(d) || d > curbConfig.maxDetectionRangeMeters) {
                    continue;
                }
                min = Math.min(min, d);
                max = Math.max(max, d);
                validCount++;
            }
            if (validCount == 0) {
                lastVehiclePoints = List.of();
                return CurbDetectionResult.noDetection(timestampMs, camera.role);
            }

            List<CurbZoneSegment> zones = zoneClassifier.classify(tracked, distances);
            double confidence = detConf[0] * trackerConfidence(tracker.status());
            boolean valid = confidence >= curbConfig.minConfidence;

            return new CurbDetectionResult(
                    timestampMs,
                    camera.role,
                    true,
                    confidence,
                    tracker.status(),
                    tracked,
                    min,
                    max,
                    zones,
                    valid);
        }

        private static double trackerConfidence(TrackingStatus status) {
            switch (status) {
                case TRACKING:
                    return 1.0;
                case ACQUIRED:
                    return 0.85;
                case PREDICTED:
                    return 0.6;
                default:
                    return 0.3;
            }
        }
    }
}
