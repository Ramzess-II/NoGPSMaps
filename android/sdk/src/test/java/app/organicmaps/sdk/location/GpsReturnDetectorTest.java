package app.organicmaps.sdk.location;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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
      back = detector.onGpsPosition(lat + offsetLat, LON, 10, startMs + i * 1000L, lat, LON, ownErrorM);
    }
    return back;
  }

  @Test
  public void trustsGpsAfterStreak()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 14, 0, 20));
    assertTrue(detector.onGpsPosition(LAT + 15 * STEP, LON, 10, 15_000, LAT + 15 * STEP, LON, 20));
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
    assertTrue(detector.onGpsPosition(LAT + 15 * STEP, LON, 10, 30_000, LAT + 15 * STEP, LON, 20));
  }

  @Test
  public void restartsStreakAfterJump()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 10, 0, NO_OWN));
    // 2 km in a second.
    assertFalse(detector.onGpsPosition(LAT + 0.02, LON, 10, 11_000, 0, 0, NO_OWN));
    assertFalse(detector.onGpsPosition(LAT + 0.02, LON, 10, 12_000, 0, 0, NO_OWN));
  }

  @Test
  public void distrustsInaccurateGps()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    for (int i = 0; i <= 30; i++)
      assertFalse(detector.onGpsPosition(LAT, LON, 60, i * 1000L, LAT, LON, 20));
  }

  @Test
  public void ignoresInaccurateGpsBetweenAccurate()
  {
    // GPS and fused positions come together, the fused ones are rough.
    final GpsReturnDetector detector = new GpsReturnDetector();
    boolean back = false;
    for (int i = 0; i <= 15; i++)
    {
      assertFalse(detector.onGpsPosition(LAT, LON, 60, i * 1000L, LAT, LON, 20));
      back = detector.onGpsPosition(LAT, LON, 10, i * 1000L, LAT, LON, 20);
    }
    assertTrue(back);
  }

  @Test
  public void restartsStreakWhenOnlyInaccurateGps()
  {
    final GpsReturnDetector detector = new GpsReturnDetector();
    assertFalse(drive(detector, 0, 10, 0, NO_OWN));
    for (int i = 11; i <= 20; i++)
      assertFalse(detector.onGpsPosition(LAT + 10 * STEP, LON, 60, i * 1000L, 0, 0, NO_OWN));
    assertFalse(drive(detector, 21_000, 10, 0, NO_OWN));
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
