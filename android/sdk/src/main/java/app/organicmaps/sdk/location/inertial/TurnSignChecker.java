package app.organicmaps.sdk.location.inertial;

/**
 * Checks while GPS works that the gyroscope turns the car the same way as GPS does. A sensor box can be mounted
 * in any position, and a driver of its gyroscope with wrong axes turns the car left on right turns. It would be
 * seen only when GPS is lost, so the turns are compared beforehand.
 */
public class TurnSignChecker
{
  public enum Result
  {
    UNKNOWN,
    OK,
    REVERSED,
  }

  // A turn counts when GPS has turned the car at least this much, the gyroscope at least half of it.
  static final double MIN_TURN_DEG = 30;
  // The turn ends when GPS goes straight again.
  static final double MAX_STRAIGHT_STEP_DEG = 3;
  // GPS headings with a longer gap between them don't make a turn.
  static final long MAX_GPS_GAP_MS = 3000;
  static final int TURNS_TO_DECIDE = 3;

  private double mLastBearingDeg = Double.NaN;
  private long mLastGpsMs;
  private double mGpsTurnDeg;
  private double mGyroTurnDeg;
  private int mSameTurns;
  private int mOppositeTurns;
  private Result mResult = Result.UNKNOWN;

  /**
   * @param deltaDeg the clockwise rotation of the car by the gyroscope while it drives.
   */
  public void onGyro(double deltaDeg)
  {
    if (!Double.isNaN(mLastBearingDeg))
      mGyroTurnDeg += deltaDeg;
  }

  /**
   * @param bearingDeg a reliable GPS heading of the driving car.
   * @return the result if it has changed by this heading, null otherwise.
   */
  public Result onGpsBearing(double bearingDeg, long timeMs)
  {
    if (Double.isNaN(mLastBearingDeg) || timeMs - mLastGpsMs > MAX_GPS_GAP_MS)
    {
      restart(bearingDeg, timeMs);
      return null;
    }
    final double step = DeadReckoning.angleDiff(bearingDeg, mLastBearingDeg);
    mLastBearingDeg = bearingDeg;
    mLastGpsMs = timeMs;
    mGpsTurnDeg += step;
    if (Math.abs(step) > MAX_STRAIGHT_STEP_DEG)
      return null;

    // Driving straight: a turn has ended, or the gyroscope drift is dropped.
    final Result old = mResult;
    if (Math.abs(mGpsTurnDeg) >= MIN_TURN_DEG && Math.abs(mGyroTurnDeg) >= MIN_TURN_DEG / 2)
    {
      if (Math.signum(mGpsTurnDeg) == Math.signum(mGyroTurnDeg))
        mSameTurns++;
      else
        mOppositeTurns++;
      if (mOppositeTurns >= TURNS_TO_DECIDE && mOppositeTurns >= TURNS_TO_DECIDE * mSameTurns)
        mResult = Result.REVERSED;
      else if (mSameTurns >= TURNS_TO_DECIDE && mSameTurns >= TURNS_TO_DECIDE * mOppositeTurns)
        mResult = Result.OK;
    }
    mGpsTurnDeg = 0;
    mGyroTurnDeg = 0;
    return mResult != old ? mResult : null;
  }

  /**
   * GPS has no reliable heading now, e.g. the car stands still or GPS is lost.
   */
  public void onNoGpsBearing()
  {
    mLastBearingDeg = Double.NaN;
  }

  public Result getResult()
  {
    return mResult;
  }

  public int getSameTurns()
  {
    return mSameTurns;
  }

  public int getOppositeTurns()
  {
    return mOppositeTurns;
  }

  private void restart(double bearingDeg, long timeMs)
  {
    mLastBearingDeg = bearingDeg;
    mLastGpsMs = timeMs;
    mGpsTurnDeg = 0;
    mGyroTurnDeg = 0;
  }
}
