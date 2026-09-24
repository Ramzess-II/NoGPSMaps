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
  public void angleDiff()
  {
    assertEquals(20, DeadReckoning.angleDiff(10, 350), 1e-9);
    assertEquals(-20, DeadReckoning.angleDiff(350, 10), 1e-9);
    assertEquals(0, DeadReckoning.angleDiff(90, 90), 1e-9);
  }

  @Test
  public void snapsToRoadAndCorrectsHeading()
  {
    final DeadReckoning dr = new DeadReckoning();
    dr.setPosition(LAT, LON);
    dr.setHeading(10);
    // The road goes to the north: the position moves to it, the heading turns 30% towards it.
    assertTrue(dr.snapToRoad(LAT + 0.0001, LON + 0.0002, 0));
    assertEquals(LAT + 0.0001, dr.getLat(), 1e-12);
    assertEquals(LON + 0.0002, dr.getLon(), 1e-12);
    assertEquals(7, dr.getHeading(), 1e-9);
    // The same road in the opposite direction.
    assertTrue(dr.snapToRoad(LAT, LON, 180));
    assertEquals(7 - 7 * DeadReckoning.SNAP_HEADING_WEIGHT, dr.getHeading(), 1e-9);
  }

  @Test
  public void ignoresCrossingRoad()
  {
    final DeadReckoning dr = new DeadReckoning();
    dr.setPosition(LAT, LON);
    dr.setHeading(0);
    assertFalse(dr.snapToRoad(LAT + 0.001, LON, 90));
    assertEquals(LAT, dr.getLat(), 0);
    assertEquals(0, dr.getHeading(), 0);
  }

  @Test
  public void movesByBearing()
  {
    final double[] north = DeadReckoning.move(LAT, LON, 0, 500);
    assertEquals(500, (north[0] - LAT) * M_PER_DEG, 1);
    assertEquals(LON, north[1], 1e-9);

    final double[] back = DeadReckoning.move(north[0], north[1], 180, 500);
    assertEquals(LAT, back[0], 1e-6);
    assertEquals(LON, back[1], 1e-9);
  }

  @Test
  public void estimatesSpeedScaleFromCorrections()
  {
    final SpeedScale scale = new SpeedScale();
    assertEquals(1, scale.get(), 0);

    // A correction on a short distance says nothing about the speed.
    scale.onDistance(100);
    scale.onCorrection(50);
    assertEquals(1, scale.get(), 0);

    // The position lagged by 150 m on 1000 m, a half of the error is compensated.
    scale.onDistance(900);
    scale.onCorrection(100);
    assertEquals(1 + 150.0 / 1000 * SpeedScale.CORRECTION_WEIGHT, scale.get(), 1e-9);

    // The position ran ahead, the speed is lowered.
    final double overestimated = scale.get();
    scale.onDistance(1000);
    scale.onCorrection(-100);
    assertTrue(scale.get() < overestimated);

    scale.reset();
    assertEquals(1, scale.get(), 0);
  }

  @Test
  public void limitsSpeedScale()
  {
    final SpeedScale scale = new SpeedScale();
    for (int i = 0; i < 100; i++)
    {
      scale.onDistance(1000);
      scale.onCorrection(1000);
    }
    assertEquals(SpeedScale.MAX_SCALE, scale.get(), 0);

    for (int i = 0; i < 100; i++)
    {
      scale.onDistance(1000);
      scale.onCorrection(-1000);
    }
    assertEquals(SpeedScale.MIN_SCALE, scale.get(), 0);
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
