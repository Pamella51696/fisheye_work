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

    public PanoramaSettings(int panelWidth, int panelHeight,
                            double panelYawDeg, double panelPitchDeg,
                            double horizonFraction, double maxIncidenceDeg,
                            int overlapPx, int edgeFeatherPx) {
        this.panelWidth = panelWidth;
        this.panelHeight = panelHeight;
        this.panelYawDeg = panelYawDeg;
        this.panelPitchDeg = panelPitchDeg;
        this.horizonFraction = horizonFraction;
        this.maxIncidenceDeg = maxIncidenceDeg;
        this.overlapPx = overlapPx;
        this.edgeFeatherPx = edgeFeatherPx;
    }
}
