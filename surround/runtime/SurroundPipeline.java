package surround.runtime;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import surround.calibration.CameraConfig;
import surround.calibration.RigConfig;
import surround.calibration.RigConfigLoader;
import surround.stitching.PanoramaBlender;
import surround.stitching.PanoramaMapper;
import surround.undistort.RectilinearMapper;
import surround.undistort.RectilinearSampling;

/**
 * Runtime: optional mild undistort → rays → vehicle frame → panorama → blend.
 */
public final class SurroundPipeline {

    private final RigConfig rig;
    private final List<PanoramaMapper> panoramaMappers;
    private final List<RectilinearMapper> rectifyMappers;
    private final List<RectilinearMapper> stitchUndistortMappers;
    private final PanoramaBlender blender;
    private final Mat[] panelScratch;
    private final Mat[] rectScratch;
    private final boolean partialUndistort;

    public SurroundPipeline(RigConfig rig) {
        this.rig = rig;
        this.partialUndistort = rig.panorama.partialUndistortForStitch;
        this.panoramaMappers = new ArrayList<>();
        this.rectifyMappers = new ArrayList<>();
        this.stitchUndistortMappers = new ArrayList<>();
        for (CameraConfig cam : rig.cameras) {
            rectifyMappers.add(new RectilinearMapper(
                    cam.intrinsics,
                    cam.intrinsics.rectBalance,
                    cam.intrinsics.rectOutputFovDeg));
            RectilinearMapper stitchRect = new RectilinearMapper(
                    cam.intrinsics,
                    rig.panorama.stitchUndistortBalance,
                    rig.panorama.stitchRectFovDeg);
            stitchUndistortMappers.add(stitchRect);
            RectilinearSampling sampling = null;
            if (partialUndistort) {
                sampling = stitchRect.samplingForSize(cam.intrinsics.width, cam.intrinsics.height);
            }
            panoramaMappers.add(new PanoramaMapper(cam, rig.panorama, sampling));
        }
        this.blender = new PanoramaBlender(rig.panorama);
        this.panelScratch = new Mat[rig.cameras.size()];
        this.rectScratch = new Mat[rig.cameras.size()];
        for (int i = 0; i < panelScratch.length; i++) {
            panelScratch[i] = new Mat(rig.panorama.panelHeight, rig.panorama.panelWidth, CvType.CV_8UC3);
            rectScratch[i] = new Mat();
        }
    }

    public static SurroundPipeline load(Path rigJson) throws IOException {
        RigConfig rig = RigConfigLoader.load(rigJson);
        System.out.println("Loaded rig (" + rig.calibrationMode + "): " + rig.description);
        if (rig.panorama.partialUndistortForStitch) {
            System.out.println("  stitch path: partial undistort balance="
                    + rig.panorama.stitchUndistortBalance
                    + " rectFov=" + rig.panorama.stitchRectFovDeg + "°");
        }
        for (CameraConfig c : rig.cameras) {
            System.out.println("  " + c.role + " model=" + c.intrinsics.model
                    + " fx=" + String.format("%.1f", c.intrinsics.fx)
                    + " cx=" + String.format("%.1f", c.intrinsics.cx)
                    + " yaw=" + c.extrinsics.effectiveYawDeg());
        }
        return new SurroundPipeline(rig);
    }

    public RigConfig rig() {
        return rig;
    }

    public Mat stitchPanorama(Mat[] fisheyeFrames) {
        if (fisheyeFrames.length != panoramaMappers.size()) {
            throw new IllegalArgumentException("expected " + panoramaMappers.size() + " frames");
        }
        for (int i = 0; i < fisheyeFrames.length; i++) {
            Mat source = fisheyeFrames[i];
            if (partialUndistort) {
                stitchUndistortMappers.get(i).rectify(source, rectScratch[i]);
                source = rectScratch[i];
            }
            panoramaMappers.get(i).project(source, panelScratch[i]);
        }
        return blender.stitch(panelScratch);
    }

    public void rectify(int cameraIndex, Mat fisheye, Mat rectilinear) {
        rectifyMappers.get(cameraIndex).rectify(fisheye, rectilinear);
    }

    public int cameraIndex(String role) {
        for (int i = 0; i < rig.cameraOrder.size(); i++) {
            if (rig.cameraOrder.get(i).equalsIgnoreCase(role)) {
                return i;
            }
        }
        return -1;
    }
}
