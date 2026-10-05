package fisheye;

import java.util.Locale;

/**
 * Ideal fisheye radius laws and named lens families used for direct ray→pixel mapping.
 * Kannala–Brandt is the default polynomial wrapper around the equidistant base.
 */
public enum FisheyeProjection {

    /** r = f·θ (OpenCV fisheye with k₁…k₄ = 0). */
    EQUIDISTANT,
    /** r = 2f·sin(θ/2). */
    EQUISOLID,
    /** r = 2f·tan(θ/2). */
    STEREOGRAPHIC,
    /** θ_d = θ·(1 + k₁θ² + …); OpenCV {@code cv2.fisheye}. */
    KANNALA_BRANDT,
    /**
     * Unified camera model (Mei / Gao): ξ lifts the sphere; good mid-wide FOV.
     * Extra calib field: {@code xi} (default 0).
     */
    MEI,
    /**
     * Double sphere (Usenko et al.): ξ and α in (0,1). Use past ~190° effective FOV.
     * Calib fields: {@code xi}, {@code alpha} (default 0.5).
     */
    DOUBLE_SPHERE;

    public static FisheyeProjection parse(String raw) {
        if (raw == null || raw.isBlank()) {
            return KANNALA_BRANDT;
        }
        String k = raw.trim().toUpperCase(Locale.ROOT).replace('-', '_').replace(' ', '_');
        if (k.equals("KB") || k.equals("OPENCV_FISHEYE")) {
            return KANNALA_BRANDT;
        }
        if (k.equals("EQUIDISTANCE")) {
            return EQUIDISTANT;
        }
        try {
            return valueOf(k);
        } catch (IllegalArgumentException e) {
            System.err.println("Unknown fisheye projection '" + raw + "', using KANNALA_BRANDT");
            return KANNALA_BRANDT;
        }
    }
}
