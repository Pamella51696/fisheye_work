package output;

import java.util.List;

public final class CurbZoneSegment {
    public final CurbZoneType type;
    public final double distanceMin;
    public final double distanceMax;
    public final List<ImagePoint> points;

    public CurbZoneSegment(CurbZoneType type,
                           double distanceMin,
                           double distanceMax,
                           List<ImagePoint> points) {
        this.type = type;
        this.distanceMin = distanceMin;
        this.distanceMax = distanceMax;
        this.points = points;
    }
}
