package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SpeedTableTest
{
  // Drives at a steady speed for |seconds|, GPS tells the real speed.
  private static void drive(SpeedTable table, double carKmh, double realKmh, int seconds)
  {
    for (int i = 0; i < seconds; i++)
      table.onSample(carKmh, realKmh / 3.6, 1, 0);
  }

  @Test
  public void unknownWithoutSamples()
  {
    assertTrue(Double.isNaN(new SpeedTable().getScale(50)));
  }

  @Test
  public void measuresEveryRange()
  {
    final SpeedTable table = new SpeedTable();
    // The speedometer is 5% optimistic at 50 km/h and 2% at 90 km/h.
    drive(table, 52.5, 50, 60);
    drive(table, 91.8, 90, 60);
    assertEquals(50 / 52.5, table.getScale(55), 1e-9);
    assertEquals(90 / 91.8, table.getScale(95), 1e-9);
    assertEquals(2, table.getKnownRanges());
  }

  @Test
  public void interpolatesUnknownRanges()
  {
    final SpeedTable table = new SpeedTable();
    drive(table, 30, 27, 60);
    drive(table, 70, 70, 60);
    // 50 km/h is in the middle between the measured 30 and 70 km/h ranges.
    assertEquals((0.9 + 1.0) / 2, table.getScale(50), 1e-9);
    // Outside the measured ranges the closest one is used.
    assertEquals(0.9, table.getScale(10), 1e-9);
    assertEquals(1.0, table.getScale(120), 1e-9);
  }

  @Test
  public void needsDistance()
  {
    final SpeedTable table = new SpeedTable();
    // 100 m only.
    drive(table, 36, 30, 10);
    assertTrue(Double.isNaN(table.getScale(36)));
  }

  @Test
  public void ignoresBadSamples()
  {
    final SpeedTable table = new SpeedTable();
    for (int i = 0; i < 100; i++)
    {
      // Too slow, accelerating, after a pause.
      table.onSample(3, 1, 1, 0);
      table.onSample(50, 20, 1, 2);
      table.onSample(50, 20, 5, 0);
    }
    assertEquals(0, table.getKnownRanges());
  }

  @Test
  public void forgetsOldSamples()
  {
    final SpeedTable table = new SpeedTable();
    drive(table, 60, 54, 600);
    // Much longer with the new tyres: the old ratio fades away.
    drive(table, 60, 60, 3000);
    assertEquals(1.0, table.getScale(60), 0.02);
  }

  @Test
  public void keepsBetweenTrips()
  {
    final SpeedTable table = new SpeedTable();
    drive(table, 52.5, 50, 60);
    final SpeedTable restored = new SpeedTable();
    restored.deserialize(table.serialize());
    assertEquals(table.getScale(50), restored.getScale(50), 1e-3);

    restored.deserialize("broken");
    assertEquals(0, restored.getKnownRanges());
  }
}
