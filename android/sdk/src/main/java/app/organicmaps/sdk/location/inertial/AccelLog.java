package app.organicmaps.sdk.location.inertial;

import androidx.annotation.Nullable;
import java.util.Locale;

/**
 * Packs the accelerometer for the trip log: one line a second, SLICES parts of it each with the mean
 * acceleration along the axes of the sensor and the jolt, i.e. how far the acceleration went from that mean.
 * The log is to check on real trips what the accelerometer can tell: braking before the car reports it,
 * bumps and rails as known places of the map.
 */
public final class AccelLog
{
  static final int SLICES = 5;
  private static final long SLICE_NS = 1_000_000_000L / SLICES;

  private final StringBuilder mLine = new StringBuilder();
  private int mSlices;
  private long mSliceStartNs;
  private int mCount;
  private final double[] mSum = new double[3];
  private final double[] mMin = new double[3];
  private final double[] mMax = new double[3];
  private double mJolt;

  /**
   * @param x, y, z the acceleration, m/s².
   * @return the line of the second that has just ended, null until then.
   */
  @Nullable
  public String onSample(long timestampNs, float x, float y, float z)
  {
    return onSample(timestampNs, x, y, z, 0);
  }

  /**
   * @param jolt the jolt during the sample measured by the sensor itself, which reads faster.
   */
  @Nullable
  public String onSample(long timestampNs, double x, double y, double z, double jolt)
  {
    String line = null;
    // A break of the sensor starts everything anew.
    if (mCount > 0 && (timestampNs < mSliceStartNs || timestampNs - mSliceStartNs > 2 * SLICE_NS))
      reset();
    if (mCount > 0 && timestampNs - mSliceStartNs >= SLICE_NS)
    {
      endSlice();
      if (mSlices == SLICES)
      {
        line = mLine.toString();
        mLine.setLength(0);
        mSlices = 0;
      }
    }
    if (mCount == 0)
      mSliceStartNs = timestampNs;
    final double[] sample = {x, y, z};
    mJolt = mCount == 0 ? jolt : Math.max(mJolt, jolt);
    for (int i = 0; i < 3; i++)
    {
      mSum[i] += sample[i];
      mMin[i] = mCount == 0 ? sample[i] : Math.min(mMin[i], sample[i]);
      mMax[i] = mCount == 0 ? sample[i] : Math.max(mMax[i], sample[i]);
    }
    mCount++;
    return line;
  }

  public void reset()
  {
    mLine.setLength(0);
    mSlices = 0;
    clearSlice();
  }

  private void clearSlice()
  {
    mCount = 0;
    for (int i = 0; i < 3; i++)
      mSum[i] = 0;
  }

  private void endSlice()
  {
    double jolt = mJolt;
    for (int i = 0; i < 3; i++)
    {
      final double mean = mSum[i] / mCount;
      jolt = Math.max(jolt, Math.max(mMax[i] - mean, mean - mMin[i]));
    }
    if (mSlices > 0)
      mLine.append(' ');
    mLine.append(String.format(Locale.US, "%.2f,%.2f,%.2f,%.1f", mSum[0] / mCount, mSum[1] / mCount,
                               mSum[2] / mCount, jolt));
    mSlices++;
    clearSlice();
  }
}
