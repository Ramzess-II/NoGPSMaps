package app.organicmaps.sdk.location.inertial;

/**
 * The speed from the car differs from the real one: the speedometer is rounded and usually optimistic, and
 * the road is not flat. The difference is estimated from the corrections the user applies to the calculated
 * position and is compensated. The speedometer error stays the same, so the estimation is kept between trips.
 */
public class SpeedScale
{
  // The error is estimated on a long enough distance only, otherwise a single correction defines it.
  static final double MIN_DISTANCE_M = 300;
  static final double MIN_SCALE = 0.85;
  static final double MAX_SCALE = 1.2;
  // A correction is rough (the user taps a button by eye), so only a part of it is taken into account.
  static final double CORRECTION_WEIGHT = 0.5;

  private double mScale = 1;
  private double mDistanceM;
  private double mCorrectionM;

  /**
   * The calculated position has moved by the distance.
   */
  public void onDistance(double distanceM)
  {
    mDistanceM += distanceM;
  }

  /**
   * The user has moved the position along the road, because it lagged behind the car.
   * @param correctionM the distance the position was moved by, negative if it was moved back.
   */
  public void onCorrection(double correctionM)
  {
    mCorrectionM += correctionM;
    if (mDistanceM < MIN_DISTANCE_M)
      return;

    final double corrected = mScale * (1 + mCorrectionM / mDistanceM * CORRECTION_WEIGHT);
    mScale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, corrected));
    mDistanceM = 0;
    mCorrectionM = 0;
  }

  /**
   * @return the ratio the speed from the car is multiplied by.
   */
  public double get()
  {
    return mScale;
  }

  /**
   * Restores the ratio estimated before, e.g. on a previous trip.
   */
  public void set(double scale)
  {
    mScale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, scale));
    mDistanceM = 0;
    mCorrectionM = 0;
  }

  public void reset()
  {
    mScale = 1;
    mDistanceM = 0;
    mCorrectionM = 0;
  }
}
