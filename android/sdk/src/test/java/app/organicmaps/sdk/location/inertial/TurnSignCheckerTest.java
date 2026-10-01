package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class TurnSignCheckerTest
{
  private long mTimeMs;
  private double mBearingDeg;

  /**
   * Drives a turn by GPS once a second, the gyroscope turns the car by gyroSign times as much.
   * @return the last result of the checker.
   */
  private TurnSignChecker.Result turn(TurnSignChecker checker, double turnDeg, double gyroSign)
  {
    TurnSignChecker.Result result = null;
    // Straight before the turn.
    for (int i = 0; i < 3; i++)
      result = drive(checker, 0, gyroSign);
    for (int i = 0; i < 6; i++)
      result = drive(checker, turnDeg / 6, gyroSign);
    for (int i = 0; i < 3; i++)
    {
      final TurnSignChecker.Result r = drive(checker, 0, gyroSign);
      if (r != null)
        result = r;
    }
    return result;
  }

  private TurnSignChecker.Result drive(TurnSignChecker checker, double stepDeg, double gyroSign)
  {
    // A small drift of the gyroscope on straight roads too.
    checker.onGyro(gyroSign * stepDeg + 0.1);
    mBearingDeg += stepDeg;
    mTimeMs += 1000;
    return checker.onGpsBearing(mBearingDeg, mTimeMs);
  }

  @Test
  public void confirmsGyroscopeTurningAsGps()
  {
    final TurnSignChecker checker = new TurnSignChecker();
    turn(checker, 90, 1);
    turn(checker, -90, 1);
    assertEquals(TurnSignChecker.Result.OK, turn(checker, 90, 1));
    assertEquals(3, checker.getSameTurns());
  }

  @Test
  public void detectsReversedGyroscope()
  {
    final TurnSignChecker checker = new TurnSignChecker();
    assertNull(turn(checker, 90, -1));
    assertNull(turn(checker, -45, -1));
    assertEquals(TurnSignChecker.Result.REVERSED, turn(checker, 90, -1));
  }

  @Test
  public void ignoresDriftOnStraightRoads()
  {
    final TurnSignChecker checker = new TurnSignChecker();
    for (int i = 0; i < 600; i++)
      drive(checker, 0, 1);
    assertEquals(TurnSignChecker.Result.UNKNOWN, checker.getResult());
    assertEquals(0, checker.getSameTurns() + checker.getOppositeTurns());
  }

  @Test
  public void ignoresTurnsWithoutGps()
  {
    final TurnSignChecker checker = new TurnSignChecker();
    drive(checker, 0, 1);
    // GPS is lost in the middle of a turn: the turn doesn't count.
    checker.onNoGpsBearing();
    checker.onGyro(-90);
    mTimeMs += 10_000;
    mBearingDeg += 90;
    for (int i = 0; i < 3; i++)
      drive(checker, 0, 1);
    assertEquals(0, checker.getSameTurns() + checker.getOppositeTurns());
  }
}
