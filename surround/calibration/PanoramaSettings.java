package surround.calibration;

public final class PanoramaSettings {

    public final int panelWidth;
    public final int panelHeight;
    public final double panelYawDeg;
    public final double panelPitchDeg;
    public final double horizonFraction;
    public final double maxIncidenceDeg;
    public final int overlapPx;
    public final int edgeFeatherPx;
    /** When true, rows below the horizon sample rays via the shared ground plane. */
    public final boolean groundPlaneEnabled;
    public final double cameraHeightZ;
    public final double groundPlaneZ;
    public final double groundDistanceNear;
    public final double groundDistanceFar;
    public final int groundBlendRows;

    public PanoramaSettings(int panelWidth, int panelHeight,
                            double panelYawDeg, double panelPitchDeg,
                            double horizonFraction, double maxIncidenceDeg,
                            int overlapPx, int edgeFeatherPx,
                            boolean groundPlaneEnabled,
                            double cameraHeightZ, double groundPlaneZ,
                            double groundDistanceNear, double groundDistanceFar,
                            int groundBlendRows) {
        this.panelWidth = panelWidth;
        this.panelHeight = panelHeight;
        this.panelYawDeg = panelYawDeg;
        this.panelPitchDeg = panelPitchDeg;
        this.horizonFraction = horizonFraction;
        this.maxIncidenceDeg = maxIncidenceDeg;
        this.overlapPx = overlapPx;
        this.edgeFeatherPx = edgeFeatherPx;
        this.groundPlaneEnabled = groundPlaneEnabled;
        this.cameraHeightZ = cameraHeightZ;
        this.groundPlaneZ = groundPlaneZ;
        this.groundDistanceNear = groundDistanceNear;
        this.groundDistanceFar = groundDistanceFar;
        this.groundBlendRows = groundBlendRows;
    }
}
