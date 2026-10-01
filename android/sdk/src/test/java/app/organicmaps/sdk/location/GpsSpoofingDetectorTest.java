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
    assertTrue(detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 70, 0));
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 1000));
    // A Wi-Fi point 8 km away, reported by two providers.
    assertFalse(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, 1500));
    assertFalse(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, 1500));
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 2000));
    // Another point near it confirms it: GPS is spoofed.
    assertTrue(detector.onNetworkPosition(KYIV_LAT + 71 * STEP, KYIV_LON, 200, 2500));
    assertFalse(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 3000));
  }

  @Test
  public void ignoresCellTowerAtWrongPoint()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    assertTrue(detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 90, 0));
    // The same wrong point 8 km away again and again: the phone lies still, the marker must not fly there.
    long time = 0;
    for (int i = 0; i < 5; i++)
      assertFalse(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, time += 20_000));
    // A fused position at home is not spoofed.
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, time += 1000));
    assertTrue(detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 100, time += 8000));
    // The point jumped away and back, it is ignored later even after a long pause.
    assertFalse(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200,
                                           time += 2 * GpsSpoofingDetector.JUMP_CONFIRM_MS));
    assertFalse(detector.isSpoofed());
  }

  @Test
  public void prefersWiFiToCellTowerAtWrongPoint()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    // In the morning the first position is from a cell tower at a wrong point, then Wi-Fi at home comes.
    assertTrue(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 530, 0));
    assertTrue(detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 50, 3000));
    assertTrue(detector.checkSatellitePosition(KYIV_LAT, KYIV_LON, 4000));
  }

  @Test
  public void acceptsJumpNothingContradicts()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    assertTrue(detector.onNetworkPosition(KYIV_LAT, KYIV_LON, 90, 0));
    // Only one cell tower far away, e.g. the phone was moved while switched off.
    long time = 1000;
    assertFalse(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, time));
    assertFalse(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200, time += 20_000));
    assertTrue(detector.onNetworkPosition(KYIV_LAT + 70 * STEP, KYIV_LON, 200,
                                          time += GpsSpoofingDetector.JUMP_CONFIRM_MS));
  }

  @Test
  public void followsNetworkPositionsOfMovingCar()
  {
    final GpsSpoofingDetector detector = new GpsSpoofingDetector();
    long time = 0;
    // 60 km/h, a network position every 20 s.
    for (int i = 0; i < 20; i++)
      assertTrue(detector.onNetworkPosition(KYIV_LAT + 3 * i * STEP, KYIV_LON, 100, time += 20_000));
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
