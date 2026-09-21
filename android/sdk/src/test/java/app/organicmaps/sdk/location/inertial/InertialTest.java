package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class InertialTest
{
  private static final double LAT = 50.4501;
  private static final double LON = 30.5234;
  // Meters per degree of latitude.
  private static final double M_PER_DEG = 111_195;

  @Test
  public void parsesSpeed()
  {
    assertEquals(60, Elm327Parser.parseSpeed("41 0D 3C\r"));
    assertEquals(60, Elm327Parser.parseSpeed("410D3C"));
    assertEquals(0, Elm327Parser.parseSpeed("SEARCHING...\r41 0D 00\r\r"));
    assertEquals(255, Elm327Parser.parseSpeed("41 0d ff"));
    assertEquals(60, Elm327Parser.parseSpeed("41 0D 3C\r41 0D 3C\r"));
    assertEquals(Elm327Parser.NO_SPEED, Elm327Parser.parseSpeed("NO DATA\r"));
    assertEquals(Elm327Parser.NO_SPEED, Elm327Parser.parseSpeed("UNABLE TO CONNECT"));
    assertEquals(Elm327Parser.NO_SPEED, Elm327Parser.parseSpeed("41 0D"));
    assertEquals(Elm327Parser.NO_SPEED, Elm327Parser.parseSpeed("?"));
  }

  @Test
  public void notReadyWithoutHeading()
  {
    final DeadReckoning dr = new DeadReckoning();
    dr.setPosition(LAT, LON);
    dr.setSpeed(10);
    dr.advance(10);
    assertFalse(dr.isReady());
    assertEquals(LAT, dr.getLat(), 0);
  }

  @Test
  public void drivesNorthAndEast()
  {
    final DeadReckoning dr = new DeadReckoning();
    dr.setPosition(LAT, LON);
    dr.setHeading(0);
    dr.setSpeed(10);
    dr.advance(100);
    assertEquals(1000, (dr.getLat() - LAT) * M_PER_DEG, 1);
    assertEquals(LON, dr.getLon(), 1e-9);

    dr.setPosition(LAT, LON);
    dr.setHeading(90);
    dr.advance(100);
    assertEquals(LAT, dr.getLat(), 1e-6);
    assertEquals(1000, (dr.getLon() - LON) * M_PER_DEG * Math.cos(Math.toRadians(LAT)), 1);
  }

  @Test
  public void accuracyGrowsWithDistance()
  {
    final DeadReckoning dr = new DeadReckoning();
    dr.setPosition(LAT, LON);
    dr.setHeading(45);
    dr.setSpeed(20);
    dr.advance(50);
    assertEquals(1000, dr.getDistanceSinceFix(), 1e-6);
    assertEquals(DeadReckoning.BASE_ACCURACY_M + 1000 * DeadReckoning.ACCURACY_PER_METER, dr.getAccuracy(), 1e-6);
    dr.setPosition(LAT, LON);
    assertEquals(DeadReckoning.BASE_ACCURACY_M, dr.getAccuracy(), 1e-6);
  }

  @Test
  public void rotatesAndNormalizesHeading()
  {
    final DeadReckoning dr = new DeadReckoning();
    dr.setHeading(350);
    dr.rotate(20);
    assertEquals(10, dr.getHeading(), 1e-9);
    dr.rotate(-30);
    assertEquals(340, dr.getHeading(), 1e-9);
  }

  @Test
  public void yawRateForAnyPhoneOrientation()
  {
    final float[] noBias = {0, 0, 0};
    // Phone lies flat screen up: turning right (clockwise from above) is a negative rotation around z.
    assertEquals(10, GyroCalibrator.yawRateDeg(new float[] {0, 0, (float) Math.toRadians(-10)}, noBias,
                                               new float[] {0, 0, 1}), 1e-4);
    // Phone stands in portrait on the dashboard: up is the phone y axis.
    assertEquals(10, GyroCalibrator.yawRateDeg(new float[] {0, (float) Math.toRadians(-10), 0}, noBias,
                                               new float[] {0, 1, 0}), 1e-4);
    // Rotation around a horizontal axis (bumps) is not a turn.
    assertEquals(0, GyroCalibrator.yawRateDeg(new float[] {1, 0, 0}, noBias, new float[] {0, 1, 0}), 1e-9);
    // The bias is removed.
    assertEquals(0, GyroCalibrator.yawRateDeg(new float[] {0, 0, 0.01f}, new float[] {0, 0, 0.01f},
                                              new float[] {0, 0, 1}), 1e-9);
  }

  @Test
  public void calibratesStillPhone()
  {
    final GyroCalibrator calibrator = new GyroCalibrator();
    for (int i = 0; i < GyroCalibrator.REQUIRED_SAMPLES; i++)
    {
      final float noise = (i % 2 == 0) ? 0.005f : -0.005f;
      calibrator.add(new float[] {0.02f + noise, -0.01f, 0.003f}, new float[] {0, 9.81f + noise, 0.3f});
    }
    assertTrue(calibrator.isComplete());
    assertTrue(calibrator.isStill());
    assertEquals(0.02f, calibrator.getBias()[0], 1e-6);
    assertEquals(-0.01f, calibrator.getBias()[1], 1e-6);
    assertEquals(1, calibrator.getUp()[1], 1e-3);
  }

  @Test
  public void rejectsMovingPhone()
  {
    final GyroCalibrator calibrator = new GyroCalibrator();
    for (int i = 0; i < GyroCalibrator.REQUIRED_SAMPLES; i++)
      calibrator.add(new float[] {(i % 2 == 0) ? 0.5f : -0.5f, 0, 0}, new float[] {0, 9.81f, 0});
    assertFalse(calibrator.isStill());
  }
}
