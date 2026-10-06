package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SpeedLagTest
{
  // The car accelerates to 15 m/s in 10 s, drives 10 s, brakes in 10 s and stands 10 s.
  private static double trueSpeed(double timeSec)
  {
    final double t = ((timeSec % 40) + 40) % 40;
    if (t < 10)
      return 1.5 * t;
    if (t < 20)
      return 15;
    if (t < 30)
      return 1.5 * (30 - t);
    return 0;
  }

  /**
   * Drives for the time: the car reports its speed 5 times a second rounded to 1 km/h and late by the lag,
   * GPS measures the real speed every second.
   */
  private static void drive(SpeedLag lag, double lagSec, int seconds)
  {
    for (int ms = 0; ms < seconds * 1000; ms += 200)
    {
      lag.onCarSpeed(ms, Math.floor(trueSpeed(ms / 1000.0 - lagSec) * 3.6) / 3.6);
      if (ms % 1000 == 0)
        lag.onGpsSpeed(ms, trueSpeed(ms / 1000.0));
    }
  }

  @Test
  public void usualLagUntilMeasured()
  {
    final SpeedLag lag = new SpeedLag();
    assertFalse(lag.isMeasured());
    assertEquals(SpeedLag.DEFAULT_LAG_SEC, lag.get(), 0);
    // A steady speed tells nothing.
    for (int ms = 0; ms < 600_000; ms += 200)
    {
      lag.onCarSpeed(ms, 15);
      if (ms % 1000 == 0)
        lag.onGpsSpeed(ms, 15);
    }
    assertFalse(lag.isMeasured());
  }

  @Test
  public void measuresLag()
  {
    for (double lagSec : new double[] {0.4, 1.0, 1.4, 2.2})
    {
      final SpeedLag lag = new SpeedLag();
      drive(lag, lagSec, 400);
      assertTrue(lag.isMeasured());
      assertEquals(lagSec, lag.get(), 0.2);
    }
  }

  @Test
  public void followsChangedLag()
  {
    final SpeedLag lag = new SpeedLag();
    drive(lag, 0.5, 400);
    // Another adapter.
    drive(lag, 1.5, 6000);
    assertEquals(1.5, lag.get(), 0.2);
  }

  @Test
  public void isKeptBetweenTrips()
  {
    final SpeedLag lag = new SpeedLag();
    drive(lag, 1.6, 400);
    final SpeedLag restored = new SpeedLag();
    restored.deserialize(lag.serialize());
    assertTrue(restored.isMeasured());
    assertEquals(lag.get(), restored.get(), 0.01);
    restored.deserialize("broken");
    assertFalse(restored.isMeasured());
    restored.deserialize(lag.serialize());
    restored.clear();
    assertEquals(SpeedLag.DEFAULT_LAG_SEC, restored.get(), 0);
  }
}
