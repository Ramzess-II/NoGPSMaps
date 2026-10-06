package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import java.util.ArrayDeque;
import java.util.Locale;

/**
 * The car reports its speed about a second late: while it accelerates the calculated position falls behind
 * by the lag multiplied by the gained speed, and it runs ahead while the car brakes. The lag is measured by
 * trusted GPS while it works: the car speed is compared with the GPS speed of some time ago, the delay it
 * fits best at is the lag. It is kept between trips and compensated without GPS.
 */
public class SpeedLag
{
  // The lag of the cars measured so far, used until GPS has measured this one.
  static final double DEFAULT_LAG_SEC = 1.2;
  static final double MAX_LAG_SEC = 3;
  static final double STEP_SEC = 0.25;
  private static final int STEPS = (int) (MAX_LAG_SEC / STEP_SEC) + 1;
  // A steady speed tells nothing about the lag.
  static final double MIN_ACCELERATION_MPS2 = 0.5;
  // The lag is trusted after this many samples of a changing speed, i.e. seconds.
  static final int MIN_SAMPLES = 60;
  // Older samples are forgotten gradually, e.g. after another adapter is plugged in.
  static final int MAX_SAMPLES = 1200;
  // A longer pause between GPS samples is a break.
  static final long MAX_GPS_GAP_MS = 2000;
  private static final long HISTORY_MS = (long) (MAX_LAG_SEC * 1000) + 2000;

  // {time, speed}: the car speed since its time until the next one.
  private final ArrayDeque<double[]> mCar = new ArrayDeque<>();
  // {time, speed} of GPS waiting for the car speed of MAX_LAG_SEC later.
  private final ArrayDeque<double[]> mGps = new ArrayDeque<>();
  private long mLastGpsMs;
  private double mLastGpsSpeedMps;
  // Squared differences of the speeds for every lag.
  private final double[] mErrors = new double[STEPS];
  private double mSamples;

  /**
   * @param speedMps the car speed as the position is calculated with since timeMs.
   */
  public void onCarSpeed(long timeMs, double speedMps)
  {
    mCar.addLast(new double[] {timeMs, speedMps});
    while (mCar.size() > 1 && timeMs - mCar.peekFirst()[0] > HISTORY_MS)
      mCar.pollFirst();
    while (!mGps.isEmpty() && timeMs - mGps.peekFirst()[0] >= MAX_LAG_SEC * 1000)
      compare(mGps.pollFirst());
  }

  /**
   * @param timeMs when the speed was measured, in the time of onCarSpeed().
   */
  public void onGpsSpeed(long timeMs, double speedMps)
  {
    final long dtMs = timeMs - mLastGpsMs;
    final double acceleration = dtMs > 0 ? (speedMps - mLastGpsSpeedMps) * 1000 / dtMs : 0;
    final boolean first = mLastGpsMs == 0 || dtMs <= 0 || dtMs > MAX_GPS_GAP_MS;
    mLastGpsMs = timeMs;
    mLastGpsSpeedMps = speedMps;
    if (!first && Math.abs(acceleration) >= MIN_ACCELERATION_MPS2)
      mGps.addLast(new double[] {timeMs, speedMps});
  }

  private void compare(@NonNull double[] gps)
  {
    // The car speed must be known all the time around the sample.
    if (mCar.isEmpty() || mCar.peekFirst()[0] > gps[0])
      return;
    final double[] errors = new double[STEPS];
    for (int i = 0; i < STEPS; i++)
    {
      final double car = carSpeedAt(gps[0] + i * STEP_SEC * 1000);
      if (Double.isNaN(car))
        return;
      errors[i] = (car - gps[1]) * (car - gps[1]);
    }
    for (int i = 0; i < STEPS; i++)
      mErrors[i] += errors[i];
    if (++mSamples > MAX_SAMPLES)
    {
      mSamples /= 2;
      for (int i = 0; i < STEPS; i++)
        mErrors[i] /= 2;
    }
  }

  private double carSpeedAt(double timeMs)
  {
    double speed = Double.NaN;
    double previousMs = Double.NaN;
    for (double[] sample : mCar)
    {
      if (sample[0] > timeMs)
        break;
      speed = sample[1];
      previousMs = sample[0];
    }
    // The car speed was lost for a while.
    return timeMs - previousMs > MAX_GPS_GAP_MS ? Double.NaN : speed;
  }

  public boolean isMeasured()
  {
    return mSamples >= MIN_SAMPLES;
  }

  /**
   * @return how late the car reports its speed, seconds.
   */
  public double get()
  {
    if (!isMeasured())
      return DEFAULT_LAG_SEC;
    int best = 0;
    for (int i = 1; i < STEPS; i++)
      if (mErrors[i] < mErrors[best])
        best = i;
    if (best == 0 || best == STEPS - 1)
      return best * STEP_SEC;
    // The bottom of the parabola through the best lag and its neighbours.
    final double left = mErrors[best - 1];
    final double right = mErrors[best + 1];
    final double curve = left - 2 * mErrors[best] + right;
    final double shift = curve > 0 ? 0.5 * (left - right) / curve : 0;
    return (best + shift) * STEP_SEC;
  }

  public void clear()
  {
    mSamples = 0;
    for (int i = 0; i < STEPS; i++)
      mErrors[i] = 0;
    mGps.clear();
  }

  /**
   * @return the measurements to keep between trips.
   */
  @NonNull
  public String serialize()
  {
    final StringBuilder result = new StringBuilder(String.format(Locale.US, "%.0f", mSamples));
    for (int i = 0; i < STEPS; i++)
      result.append(String.format(Locale.US, ";%.1f", mErrors[i]));
    return result.toString();
  }

  /**
   * Restores the measurements kept before, broken ones are ignored.
   */
  public void deserialize(@NonNull String saved)
  {
    clear();
    final String[] parts = saved.split(";");
    if (parts.length != STEPS + 1)
      return;
    try
    {
      mSamples = Double.parseDouble(parts[0]);
      for (int i = 0; i < STEPS; i++)
        mErrors[i] = Double.parseDouble(parts[i + 1]);
    }
    catch (NumberFormatException e)
    {
      clear();
    }
  }
}
