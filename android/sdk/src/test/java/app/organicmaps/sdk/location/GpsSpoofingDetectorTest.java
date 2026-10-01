package app.organicmaps.sdk.location;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class GpsSpoofingDetectorTest
{
  private static final double KYIV_LAT = 50.4501;
  private static final double KYIV_LON = 30.5234;
  private static final double LIMA_LAT = -12.0453;
  private static final double LIMA_LON = -77.0569;
  // ~111 m per 0.001 degree of latitude.
  private static final double STEP = 0.001;

  @Test
  public void distance()
  {
    assertEquals(111_195, GpsSpoofingDetector.distance(0, 0, 1, 0), 10);
  }

  @Test
  public void trustsGpsNearNetworkPosition()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 70, 0);
    assertTrue(detector.checkSatellitePosition(KYIV_LAT + STEP, KYIV_LON, 1000));
    assertFalse(detector.isSpoofed());
  }

  @Test
  public void ignoresSingleWrongNetworkPosition()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 70, 0);
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 1000));
    // A Wi-Fi point 8 km away.
    detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, 1500);
    detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, 1500);
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 2000));
    // The next one confirms it: GPS is spoofed.
    detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, 2500);
    assertFalse(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 3000));
  }

  @Test
  public void detectsGpsOnAnotherContinent()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 70, 0);
    assertFalse(detector.checkSatellitePosition(LIMA_LAT, LIMA_LON, 1000));
    assertTrue(detector.isSpoofed());
  }

  @Test
  public void trustsGpsWithoutAnyReference()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 0));
  }

  @Test
  public void detectsJumpWithoutNetwork()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 0));
    // Driving: ~111 m per 5 s is ~80 km/h.
    assertTrue(detector.checkSatellitePosition(KYIV_LAT + STEP, KYIV_LON, 5000));
    // 50 km in 10 s.
    assertFalse(detector.checkSatellitePosition(KYIV_LAT + 450 * STEP, KYIV_LON, 15000));
    assertTrue(detector.isSpoofed());
  }

  @Test
  public void ignoresStaleNetworkPosition()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    detector.onNetworkPosition(LIMA_LAT, LIMA_LON, 70, 0);
    // The network position is too old and a user could drive far away since then.
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, GpsSpoofingDetector.NETWORK_MAX_AGE_MS + 1));
  }

  @Test
  public void recoversAfterConsistentPositions()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 70, 0);
    assertFalse(detector.checkSatellitePosition(LIMA_LAT, LIMA_LON, 1000));

    long time = 2000;
    for (int i = 1; i < GpsSpoofingDetector.CONSISTENT_POSITIONS_TO_RECOVER; i++)
    {
      assertFalse(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time));
      time += 1000;
    }
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time));
    assertFalse(detector.isSpoofed());
  }

  @Test
  public void flappingJammerResetsRecovery()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 70, 0);
    assertFalse(detector.checkSatellitePosition(LIMA_LAT, LIMA_LON, 1000));

    long time = 2000;
    for (int i = 1; i < GpsSpoofingDetector.CONSISTENT_POSITIONS_TO_RECOVER; i++)
      assertFalse(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time += 1000));
    assertFalse(detector.checkSatellitePosition(LIMA_LAT, LIMA_LON, time += 1000));
    assertFalse(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time += 1000));
    assertTrue(detector.isSpoofed());
  }

  @Test
  public void recoversSlowerWithoutNetwork()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 0));
    assertFalse(detector.checkSatellitePosition(LIMA_LAT, LIMA_LON, 1000));

    long time = 1000;
    for (int i = 1; i < GpsSpoofingDetector.CONSISTENT_POSITIONS_TO_RECOVER_WITHOUT_NETWORK; i++)
      assertFalse(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time += 1000));
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time += 1000));
  }
}
