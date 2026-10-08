package output;

public enum CurbZoneType {
    GREEN,
    YELLOW,
    RED,
    UNKNOWN;

    public static CurbZoneType fromDistance(double distanceMeters,
                                            double redThreshold,
                                            double yellowThreshold) {
        if (distanceMeters < 0 || Double.isNaN(distanceMeters)) {
            return UNKNOWN;
        }
        if (distanceMeters < redThreshold) {
            return RED;
        }
        if (distanceMeters < yellowThreshold) {
            return YELLOW;
        }
        return GREEN;
    }
}
