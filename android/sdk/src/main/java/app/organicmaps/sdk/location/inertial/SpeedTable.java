package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import java.util.Locale;

/**
 * How much the speed from the car differs from the real one at different speeds: the speedometer error grows
 * with the speed, and the speed is rounded to 1 km/h, which matters at low speeds. The real speed is taken from
 * trusted GPS while it works, the table is kept between trips and used without GPS.
 * <p>
 * The speeds are split into ranges, every range keeps the distance driven by the car speed and by GPS.
 */
public class SpeedTable
{
  static final int RANGE_KMH = 10;
  static final int RANGES = 14;
  // Slower speeds are rounded too roughly, the car maneuvers, and GPS speed is noisy.
  static final int MIN_SPEED_KMH = 5;
  // A range is used after this distance only: a short one is dominated by the noise.
  static final double MIN_DISTANCE_M = 300;
  // Older samples are forgotten gradually, e.g. after new tyres.
  static final double MAX_DISTANCE_M = 10_000;
  // GPS is compared with the car at a steady speed: the car reports its speed later than GPS.
  static final double MAX_ACCELERATION_MPS2 = 1;
  // A longer pause between samples is a break.
  static final double MAX_SAMPLE_DT_SEC = 2;
  static final double MIN_SCALE = 0.8;
  static final double MAX_SCALE = 1.25;

  private final double[] mCarM = new double[RANGES];
  private final double[] mGpsM = new double[RANGES];

  private static int range(double carSpeedKmh)
  {
    return Math.min(RANGES - 1, (int) (carSpeedKmh / RANGE_KMH));
  }

  /**
   * Takes into account the car speed and the real one measured at the same time.
   * @param dtSec how long the speeds lasted.
   * @param accelerationMps2 how fast the real speed changes.
   */
  public void onSample(double carSpeedKmh, double gpsSpeedMps, double dtSec, double accelerationMps2)
  {
    if (carSpeedKmh < MIN_SPEED_KMH || dtSec <= 0 || dtSec > MAX_SAMPLE_DT_SEC
        || Math.abs(accelerationMps2) > MAX_ACCELERATION_MPS2)
    {
      return;
    }
    final int i = range(carSpeedKmh);
    mCarM[i] += carSpeedKmh / 3.6 * dtSec;
    mGpsM[i] += gpsSpeedMps * dtSec;
    if (mCarM[i] > MAX_DISTANCE_M)
    {
      mCarM[i] /= 2;
      mGpsM[i] /= 2;
    }
  }

  private boolean isKnown(int i)
  {
    return mCarM[i] >= MIN_DISTANCE_M;
  }

  private double scaleOf(int i)
  {
    return Math.max(MIN_SCALE, Math.min(MAX_SCALE, mGpsM[i] / mCarM[i]));
  }

  /**
   * @return the ratio the car speed is multiplied by, NaN if nothing is known yet. A speed not measured yet is
   * interpolated from the closest measured ones.
   */
  public double getScale(double carSpeedKmh)
  {
    final int i = range(carSpeedKmh);
    if (isKnown(i))
      return scaleOf(i);
    int lower = i - 1;
    while (lower >= 0 && !isKnown(lower))
      lower--;
    int upper = i + 1;
    while (upper < RANGES && !isKnown(upper))
      upper++;
    if (lower < 0 && upper >= RANGES)
      return Double.NaN;
    if (lower < 0)
      return scaleOf(upper);
    if (upper >= RANGES)
      return scaleOf(lower);
    final double t = (double) (i - lower) / (upper - lower);
    return scaleOf(lower) + (scaleOf(upper) - scaleOf(lower)) * t;
  }

  /**
   * @return how many speed ranges are measured.
   */
  public int getKnownRanges()
  {
    int count = 0;
    for (int i = 0; i < RANGES; i++)
      if (isKnown(i))
        count++;
    return count;
  }

  public void clear()
  {
    for (int i = 0; i < RANGES; i++)
    {
      mCarM[i] = 0;
      mGpsM[i] = 0;
    }
  }

  /**
   * @return the table to keep between trips: the car and GPS distances of every range.
   */
  @NonNull
  public String serialize()
  {
    final StringBuilder result = new StringBuilder();
    for (int i = 0; i < RANGES; i++)
    {
      if (i > 0)
        result.append(';');
      result.append(String.format(Locale.US, "%.0f:%.0f", mCarM[i], mGpsM[i]));
    }
    return result.toString();
  }

  /**
   * Restores the table kept before, a broken one is ignored.
   */
  public void deserialize(@NonNull String table)
  {
    clear();
    final String[] ranges = table.split(";");
    if (ranges.length != RANGES)
      return;
    try
    {
      for (int i = 0; i < RANGES; i++)
      {
        final String[] distances = ranges[i].split(":");
        mCarM[i] = Double.parseDouble(distances[0]);
        mGpsM[i] = Double.parseDouble(distances[1]);
      }
    }
    catch (NumberFormatException | ArrayIndexOutOfBoundsException e)
    {
      clear();
    }
  }

  /**
   * @return the ranges for the log: speed and ratio of the measured ones.
   */
  @NonNull
  @Override
  public String toString()
  {
    final StringBuilder result = new StringBuilder();
    for (int i = 0; i < RANGES; i++)
    {
      if (!isKnown(i))
        continue;
      if (result.length() > 0)
        result.append(' ');
      result.append(String.format(Locale.US, "%d:%.3f", i * RANGE_KMH, scaleOf(i)));
    }
    return result.toString();
  }
}
