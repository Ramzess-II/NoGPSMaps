package app.organicmaps.sdk.location.inertial;

/**
 * Calibrates the phone gyroscope while the car stands still: the mean rotation rate is the gyroscope bias,
 * and the mean acceleration points up (the accelerometer measures the reaction to gravity).
 * <p>
 * The phone is fixed on the dashboard, so the "up" direction in the phone axes doesn't change after calibration,
 * and the car yaw rate is the rotation around it whatever the phone orientation is.
 */
public class GyroCalibrator
{
  // Samples needed for the calibration, ~3 s at 50 Hz.
  static final int REQUIRED_SAMPLES = 150;
  // Rotation noise of a still phone on a running engine is ~0.01 rad/s.
  static final double MAX_STILL_GYRO_STD_RAD_S = 0.03;
  static final double MAX_STILL_ACCEL_STD_MPS2 = 0.5;
  // The car creeping in a jam or maneuvering is often reported standing by the car, and its slow steady turn looks
  // like a still phone. The bias changes slowly with the temperature, so a larger change at a short stop is such a
  // turn: it would rotate the heading all the way until the next stop.
  static final double MAX_BIAS_CHANGE_DEG_S = 0.5;
  static final long LONG_STOP_MS = 15_000;

  private final double[] mGyroSum = new double[3];
  private final double[] mGyroSqSum = new double[3];
  private final double[] mAccelSum = new double[3];
  private final double[] mAccelSqSum = new double[3];
  private int mCount;

  public void reset()
  {
    for (int i = 0; i < 3; i++)
    {
      mGyroSum[i] = 0;
      mGyroSqSum[i] = 0;
      mAccelSum[i] = 0;
      mAccelSqSum[i] = 0;
    }
    mCount = 0;
  }

  public void add(float[] gyro, float[] accel)
  {
    for (int i = 0; i < 3; i++)
    {
      mGyroSum[i] += gyro[i];
      mGyroSqSum[i] += (double) gyro[i] * gyro[i];
      mAccelSum[i] += accel[i];
      mAccelSqSum[i] += (double) accel[i] * accel[i];
    }
    mCount++;
  }

  public boolean isComplete()
  {
    return mCount >= REQUIRED_SAMPLES;
  }

  public int getProgressPercent()
  {
    return Math.min(100, mCount * 100 / REQUIRED_SAMPLES);
  }

  /**
   * @return true if the phone didn't move during the calibration.
   */
  public boolean isStill()
  {
    for (int i = 0; i < 3; i++)
    {
      if (std(mGyroSum[i], mGyroSqSum[i]) > MAX_STILL_GYRO_STD_RAD_S)
        return false;
      if (std(mAccelSum[i], mAccelSqSum[i]) > MAX_STILL_ACCEL_STD_MPS2)
        return false;
    }
    return true;
  }

  public float[] getBias()
  {
    return new float[] {(float) (mGyroSum[0] / mCount), (float) (mGyroSum[1] / mCount),
                        (float) (mGyroSum[2] / mCount)};
  }

  /**
   * @return the unit vector pointing up in the phone axes.
   */
  public float[] getUp()
  {
    final double x = mAccelSum[0] / mCount;
    final double y = mAccelSum[1] / mCount;
    final double z = mAccelSum[2] / mCount;
    final double norm = Math.sqrt(x * x + y * y + z * z);
    return new float[] {(float) (x / norm), (float) (y / norm), (float) (z / norm)};
  }

  /**
   * @param oldBias the bias in use, null if there is none.
   * @param stoppedMs how long the car stands.
   * @return true if the new bias of a calibration at a stop can be trusted.
   */
  public static boolean isBiasChangeAllowed(float[] oldBias, float[] newBias, float[] up, long stoppedMs)
  {
    return oldBias == null || stoppedMs >= LONG_STOP_MS
        || Math.abs(yawRateDeg(newBias, oldBias, up)) <= MAX_BIAS_CHANGE_DEG_S;
  }

  private double std(double sum, double sqSum)
  {
    final double mean = sum / mCount;
    return Math.sqrt(Math.max(0, sqSum / mCount - mean * mean));
  }

  /**
   * @param gyro rotation rate in the phone axes, rad/s (counter-clockwise positive, as Android reports).
   * @return the car yaw rate, deg/s, clockwise positive (as the compass heading grows).
   */
  public static double yawRateDeg(float[] gyro, float[] bias, float[] up)
  {
    double dot = 0;
    for (int i = 0; i < 3; i++)
      dot += (gyro[i] - bias[i]) * up[i];
    return -Math.toDegrees(dot);
  }
}
