package app.organicmaps.sdk.location;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static java.lang.Double.NaN;

import org.junit.Test;

public class GpsReturnDetectorTest
{
  private static final double LAT = 50.4501;
  private static final double LON = 30.5234;
  // ~11 m per 0.0001 degree of latitude.
  private static final double STEP = 0.0001;
  private static final double NO_OWN = Double.NaN;

  // Feeds a GPS position every second for |seconds|, the car goes north at ~11 m/s from the own position.
  private static boolean drive(GpsReturnDetector detector, long startMs, int seconds, double offsetLat, double ownErrorM)
  {
    boolean back = false;
    for (int i = 0; i <= seconds; i++)
    {
      final double lat = LAT + i * STEP;
      back = detector.onGpsPosition(lat + offsetLat, LON, 10, startMs + i * 1000L, lat, LON, ownErrorM, NaN, NaN, false);
    }
    return back;
  }

  @Test
  public void trustsGpsAfterStreak()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 14, 0, 20));
    assertTrue(detector.onGpsPosition(LAT + 15 * STEP, LON, 10, 15_000, LAT + 15 * STEP, LON, 20, NaN, NaN, false));
  }

  @Test
  public void trustsGpsWithoutOwnPosition()
  {
    assertTrue(drive(new GpsReturnDetector(), 0, 15, 0, NO_OWN));
  }

  @Test
  public void distrustsGpsFarFromOwnPosition()
  {
    // A spoofer shifts the position by 500 m only, the cell towers would not notice it.
    assertFalse(drive(new GpsReturnDetector(), 0, 60, 500 * STEP / 11, 20));
    // An old mark: the car could have driven there.
    assertTrue(drive(new GpsReturnDetector(), 0, 15, 500 * STEP / 11, 1000));
  }

  @Test
  public void restartsStreakAfterGap()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 10, 0, 20));
    assertFalse(drive(detector, 15_000, 14, 0, 20));
    assertTrue(detector.onGpsPosition(LAT + 15 * STEP, LON, 10, 30_000, LAT + 15 * STEP, LON, 20, NaN, NaN, false));
  }

  @Test
  public void restartsStreakAfterJump()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 10, 0, NO_OWN));
    // 2 km in a second.
    assertFalse(detector.onGpsPosition(LAT + 0.02, LON, 10, 11_000, 0, 0, NO_OWN, NaN, NaN, false));
    assertFalse(detector.onGpsPosition(LAT + 0.02, LON, 10, 12_000, 0, 0, NO_OWN, NaN, NaN, false));
  }

  @Test
  public void distrustsInaccurateGps()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    for (int i = 0; i <= 30; i++)
      assertFalse(detector.onGpsPosition(LAT, LON, 60, i * 1000L, LAT, LON, 20, NaN, NaN, false));
  }

  @Test
  public void ignoresInaccurateGpsBetweenAccurate()
  {
    // GPS and fused positions come together, the fused ones are rough.
    final GpsReturnDetector detector = new GpsReturnDetector();
    boolean back = false;
    for (int i = 0; i <= 15; i++)
    {
      assertFalse(detector.onGpsPosition(LAT, LON, 60, i * 1000L, LAT, LON, 20, NaN, NaN, false));
      back = detector.onGpsPosition(LAT, LON, 10, i * 1000L, LAT, LON, 20, NaN, NaN, false);
    }
    assertTrue(back);
  }

  @Test
  public void restartsStreakWhenOnlyInaccurateGps()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 10, 0, NO_OWN));
    for (int i = 11; i <= 20; i++)
      assertFalse(detector.onGpsPosition(LAT + 10 * STEP, LON, 60, i * 1000L, 0, 0, NO_OWN, NaN, NaN, false));
    assertFalse(drive(detector, 21_000, 10, 0, NO_OWN));
  }

  // Feeds GPS positions 500 m away from a wrong mark while the car goes north at ~11 m/s.
  private static boolean driveAwayFromMark(GpsReturnDetector detector, int seconds, double carSpeedMps,
                                           boolean onRoad)
  {
    boolean back = false;
    for (int i = 0; i <= seconds; i++)
    {
      final double lat = LAT + i * STEP;
      back = detector.onGpsPosition(lat + 500 * STEP / 11, LON, 10, i * 1000L, lat, LON, 20, 11, carSpeedMps, onRoad);
    }
    return back;
  }

  @Test
  public void trustsGpsFollowingCar()
  {
    // The mark is wrong, GPS is right: it goes along the roads as fast as the car.
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(driveAwayFromMark(detector, 29, 11.5, true));
    assertTrue(detector.onGpsPosition(LAT + 30 * STEP + 500 * STEP / 11, LON, 10, 30_000, LAT + 30 * STEP, LON, 20, 11,
                                      11.5, true));
  }

  @Test
  public void trustsGpsFollowingCarInJam()
  {
    // The car creeps at ~2 m/s: 120 m in a minute.
    final GpsReturnDetector detector = new GpsReturnDetector();
    boolean back = false;
    for (int i = 0; i <= 60 && !back; i++)
    {
      final double lat = LAT + i * STEP * 2 / 11;
      back = detector.onGpsPosition(lat + 500 * STEP / 11, LON, 10, i * 1000L, lat, LON, 20, 2, 2.2, true);
      assertTrue(back || i < 50);
    }
    assertTrue(back);
  }

  @Test
  public void distrustsGpsNotFollowingCar()
  {
    // A spoofer moves the position while the car stands or drives slower.
    assertFalse(driveAwayFromMark(new GpsReturnDetector(), 60, 0, true));
    assertFalse(driveAwayFromMark(new GpsReturnDetector(), 60, 5, true));
    assertFalse(driveAwayFromMark(new GpsReturnDetector(), 60, NaN, true));
    // It leads the position off the roads.
    assertFalse(driveAwayFromMark(new GpsReturnDetector(), 60, 11, false));
  }

  @Test
  public void restartsStreakAfterReset()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 10, 0, 20));
    detector.reset();
    assertFalse(drive(detector, 11_000, 10, 0, 20));
  }
}
