package com.middleware.panorama.wp5;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Pure-Java tests for Package 2 (curb / obstacle zones + panorama mapping). */
class CurbModelTest {

    @BeforeAll static void defaultsOnly() { System.setProperty("wp5.profile", "none"); }

    private static final String[] CAMS = {"front", "rear", "left", "right"};
    private final Wp5Config cfg = Wp5Config.load();
    private CurbModel model() { return new CurbModel(cfg, CAMS, 640, 360, 80); }

    private static String one(String id, String pos, double d) {
        return "{\"sensors\":[{\"id\":\"" + id + "\",\"pos\":\"" + pos + "\",\"distanceM\":" + d + ",\"kind\":\"CURB\"}]}";
    }

    @Test
    void classify_thresholds() {
        CurbModel m = model();
        assertEquals(Zone.DANGER,  m.classify(0.30, Zone.NONE));
        assertEquals(Zone.DANGER,  m.classify(cfg.dangerM, Zone.NONE));
        assertEquals(Zone.WARNING, m.classify(0.80, Zone.NONE));
        assertEquals(Zone.SAFE,    m.classify(1.50, Zone.NONE));
        assertEquals(Zone.SAFE,    m.classify(-1, Zone.NONE));        // no echo = clear
        assertEquals(Zone.SAFE,    m.classify(cfg.maxRangeM, Zone.NONE));
    }

    @Test
    void classify_hysteresis_preventsFlicker() {
        CurbModel m = model();
        // in DANGER, distance wiggles just above threshold -> must stay DANGER
        assertEquals(Zone.DANGER, m.classify(cfg.dangerM + cfg.hysteresisM / 2, Zone.DANGER));
        // clearly above threshold + margin -> allowed to improve
        assertEquals(Zone.WARNING, m.classify(cfg.dangerM + cfg.hysteresisM + 0.01, Zone.DANGER));
        // getting worse is always immediate
        assertEquals(Zone.DANGER, m.classify(cfg.dangerM - 0.01, Zone.WARNING));
    }

    @Test
    void ingest_ignoresUnknownPositions_andCountsAccepted() {
        CurbModel m = model();
        assertEquals(0, m.ingestJson(one("X", "TOP_OF_ROOF", 1.0), 0));
        assertEquals(1, m.ingestJson(one("RC", "REAR_CENTER", 1.0), 0));
    }

    @Test
    void snapshot_mapsRearCenterIntoRearCameraStrip() {
        CurbModel m = model();
        m.ingestJson(one("RC", "REAR_CENTER", 0.3), 1000);
        String j = m.snapshot(1, 1000).json;
        // rear camera is index 1 -> x0 = 1*(640-80)=560 ; REAR_CENTER = 33%..67% of 640 => x=560+211=771, w=218
        assertTrue(j.contains("\"zone\":\"DANGER\""), j);
        assertTrue(j.contains("\"color\":\"#FF3B30\""));
        assertTrue(j.contains("\"camera\":\"rear\""));
        assertTrue(j.contains("\"pano\":{\"x\":771,\"w\":218"), j);
        assertTrue(j.contains("\"panorama\":{\"width\":2320,\"height\":360}"));
    }

    @Test
    void snapshot_staleSensor_becomesNone_notSafe() {
        CurbModel m = model();
        m.ingestJson(one("RC", "REAR_CENTER", 0.3), 1000);
        CurbModel.CurbState st = m.snapshot(1, 1000 + cfg.staleMs + 1);
        assertEquals(Zone.NONE, st.worst);
        assertTrue(st.json.contains("\"distanceM\":null"));
    }

    @Test
    void snapshot_summary_leftRightAndNearest() {
        CurbModel m = model();
        m.ingestJson("{\"sensors\":["
            + "{\"id\":\"A\",\"pos\":\"LEFT_REAR\",\"distanceM\":0.3,\"kind\":\"CURB\"},"
            + "{\"id\":\"B\",\"pos\":\"RIGHT_REAR\",\"distanceM\":1.8,\"kind\":\"CURB\"}]}", 0);
        String j = m.snapshot(1, 0).json;
        assertTrue(j.contains("\"leftWorst\":\"DANGER\""), j);
        assertTrue(j.contains("\"rightWorst\":\"SAFE\""), j);
        assertTrue(j.contains("\"nearestSensorId\":\"A\""), j);
        assertTrue(j.contains("\"worstZone\":\"DANGER\""), j);
    }

    @Test
    void signature_changesOnlyWhenVisibleStateChanges() {
        CurbModel m = model();
        m.ingestJson(one("RC", "REAR_CENTER", 1.500), 0);
        String s1 = m.snapshot(1, 0).signature;
        m.ingestJson(one("RC", "REAR_CENTER", 1.505), 10);   // < 2 cm change
        assertEquals(s1, m.snapshot(2, 10).signature);
        m.ingestJson(one("RC", "REAR_CENTER", 0.400), 20);
        assertNotEquals(s1, m.snapshot(3, 20).signature);
    }

    @Test
    void obstacleSensors_areIgnored_zonesAreCurbOnly() {
        CurbModel m = model();
        int ok = m.ingestJson("{\"sensors\":["
            + "{\"id\":\"O\",\"pos\":\"REAR_CENTER\",\"distanceM\":0.2,\"kind\":\"OBSTACLE\"},"
            + "{\"id\":\"U\",\"pos\":\"FRONT_CENTER\",\"distanceM\":0.2},"
            + "{\"id\":\"C\",\"pos\":\"LEFT_REAR\",\"distanceM\":0.8,\"kind\":\"CURB\"}]}", 0);
        assertEquals(1, ok);
        String j = m.snapshot(1, 0).json;
        assertFalse(j.contains("\"id\":\"O\""), j);
        assertFalse(j.contains("\"id\":\"U\""), j);
        assertTrue(j.contains("\"id\":\"C\""), j);
        assertTrue(j.contains("\"worstZone\":\"WARNING\""), j);   // the 0.2 m obstacle must NOT make it DANGER
    }

    @Test
    void zoneWorst_ignoresNone() {
        assertEquals(Zone.SAFE, Zone.worst(Zone.NONE, Zone.SAFE));
        assertEquals(Zone.DANGER, Zone.worst(Zone.WARNING, Zone.DANGER));
    }
}
