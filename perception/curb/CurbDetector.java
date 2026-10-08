package perception.curb;

import org.opencv.core.Mat;

import output.ImagePoint;

import java.util.List;

/** Replaceable curb / road-edge detector (classical CV or future ML). */
public interface CurbDetector {

    /**
     * Detect curb boundary polyline in rectilinear image coordinates.
     *
     * @param rectilinearBgr undistorted BGR frame
     * @param role           camera role (left, front, right, rear)
     * @param outConfidence  detector confidence in [0,1]
     * @return ordered curb points (image space), possibly empty
     */
    List<ImagePoint> detect(Mat rectilinearBgr, String role, double[] outConfidence);
}
