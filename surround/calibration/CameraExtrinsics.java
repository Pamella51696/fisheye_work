package surround.calibration;

public final class CameraExtrinsics {

    public final double yawDeg;
    public final double pitchDeg;
    public final double rollDeg;
    public final double yawOffsetDeg;
    public final double pitchOffsetDeg;

    public CameraExtrinsics(double yawDeg, double pitchDeg, double rollDeg,
                            double yawOffsetDeg, double pitchOffsetDeg) {
        this.yawDeg = yawDeg;
        this.pitchDeg = pitchDeg;
        this.rollDeg = rollDeg;
        this.yawOffsetDeg = yawOffsetDeg;
        this.pitchOffsetDeg = pitchOffsetDeg;
    }

    public double effectiveYawDeg() {
        return yawDeg + yawOffsetDeg;
    }

    public double effectivePitchDeg() {
        return pitchDeg + pitchOffsetDeg;
    }
}
