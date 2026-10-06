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

/**
 * Runtime: fisheye → rays → vehicle frame → panorama panels → blend.
 * Rectilinear corrected feeds use a separate precomputed map (debug / human view).
 */
public final class SurroundPipeline {

    private final RigConfig rig;
    private final List<PanoramaMapper> panoramaMappers;
    private final List<RectilinearMapper> rectMappers;
    private final PanoramaBlender blender;
    private final Mat[] panelScratch;

    public SurroundPipeline(RigConfig rig) {
        this.rig = rig;
        this.panoramaMappers = new ArrayList<>();
        this.rectMappers = new ArrayList<>();
        for (CameraConfig cam : rig.cameras) {
            panoramaMappers.add(new PanoramaMapper(cam, rig.panorama));
            rectMappers.add(new RectilinearMapper(
                    cam.intrinsics,
                    cam.intrinsics.rectBalance,
                    cam.intrinsics.rectOutputFovDeg));
        }
        this.blender = new PanoramaBlender(rig.panorama);
        this.panelScratch = new Mat[rig.cameras.size()];
        for (int i = 0; i < panelScratch.length; i++) {
            panelScratch[i] = new Mat(rig.panorama.panelHeight, rig.panorama.panelWidth, CvType.CV_8UC3);
        }
    }

    public static SurroundPipeline load(Path rigJson) throws IOException {
        RigConfig rig = RigConfigLoader.load(rigJson);
        System.out.println("Loaded rig (" + rig.calibrationMode + "): " + rig.description);
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

    /** {@code fisheyeFrames} in {@link RigConfig#cameraOrder} order. */
    public Mat stitchPanorama(Mat[] fisheyeFrames) {
        if (fisheyeFrames.length != panoramaMappers.size()) {
            throw new IllegalArgumentException("expected " + panoramaMappers.size() + " frames");
        }
        for (int i = 0; i < fisheyeFrames.length; i++) {
            panoramaMappers.get(i).project(fisheyeFrames[i], panelScratch[i]);
        }
        return blender.stitch(panelScratch);
    }

    public void rectify(int cameraIndex, Mat fisheye, Mat rectilinear) {
        rectMappers.get(cameraIndex).rectify(fisheye, rectilinear);
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
