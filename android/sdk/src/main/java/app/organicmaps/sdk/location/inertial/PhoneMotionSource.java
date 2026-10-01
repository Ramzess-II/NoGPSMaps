package app.organicmaps.sdk.location.inertial;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.util.log.Logger;

/**
 * The phone fixed on the dashboard: turns come from its gyroscope, the car speed comes from an ELM327
 * OBD-II Bluetooth adapter. The gyroscope is calibrated by the user and automatically on every stop.
 */
public class PhoneMotionSource implements MotionSource, SensorEventListener, Elm327Client.Listener
{
  private static final String TAG = PhoneMotionSource.class.getSimpleName();

  private static final double MAX_SENSOR_DT_SEC = 0.1;
  // Wait after the car stops before the automatic calibration, the car body still sways.
  private static final long AUTO_CALIBRATION_DELAY_MS = 2000;
  // Without fresh speed the car may move, the automatic calibration waits.
  private static final long SPEED_STALE_MS = 2000;

  @NonNull
  private final SensorManager mSensorManager;
  @NonNull
  private final Listener mListener;
  @Nullable
  private final String mElm327Address;
  @Nullable
  private Elm327Client mElm327;
  @NonNull
  private State mState = State.DISCONNECTED;

  private final GyroCalibrator mCalibrator = new GyroCalibrator();
  private final GyroCalibrator mAutoCalibrator = new GyroCalibrator();
  @NonNull
  private InertialNavigator.CalibrationState mCalibrationState = InertialNavigator.CalibrationState.NONE;
  // The car has driven during the calibration asked by the user.
  private boolean mCalibrationMoved;
  // The last automatic calibration at the current stop, it is used when the next one confirms it.
  @Nullable
  private float[] mPreviousAutoBias;
  @Nullable
  private float[] mBias;
  @Nullable
  private float[] mUp;

  private final float[] mAccel = new float[3];
  private boolean mHasAccel;
  private long mLastGyroTimestampNs;

  private int mSpeedKmh = -1;
  private long mSpeedTimeMs;
  private long mStoppedSinceMs;

  /**
   * @param elm327Address Bluetooth address of the ELM327 adapter, null if it is not chosen yet.
   */
  public PhoneMotionSource(@NonNull Context context, @Nullable String elm327Address, @NonNull Listener listener)
  {
    mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
    mElm327Address = elm327Address;
    mListener = listener;
  }

  @Override
  public void start()
  {
    Logger.i(TAG, "ELM327 = " + mElm327Address);
    Sensor gyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
    // Our own bias estimation is used, the calibrated sensor is only a fallback.
    if (gyro == null)
      gyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    final Sensor accel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    if (gyro == null || accel == null)
      Logger.e(TAG, "No gyroscope or accelerometer");
    else
    {
      mSensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME);
      mSensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_GAME);
    }

    if (mElm327Address != null)
    {
      mElm327 = new Elm327Client(mElm327Address, this);
      mElm327.start();
    }
  }

  @Override
  public void stop()
  {
    mSensorManager.unregisterListener(this);
    if (mElm327 != null)
      mElm327.stop();
    mElm327 = null;
    mState = State.DISCONNECTED;
    mSpeedKmh = -1;
    mLastGyroTimestampNs = 0;
  }

  @NonNull
  @Override
  public State getState()
  {
    return mState;
  }

  @Override
  public boolean isCalibrated()
  {
    return mBias != null && mUp != null;
  }

  @NonNull
  @Override
  public InertialNavigator.CalibrationState getCalibrationState()
  {
    return mCalibrationState;
  }

  @Override
  public int getCalibrationProgressPercent()
  {
    return mCalibrator.getProgressPercent();
  }

  @Override
  public void calibrate()
  {
    Logger.i(TAG);
    mCalibrator.reset();
    mCalibrationMoved = false;
    mCalibrationState = InertialNavigator.CalibrationState.CALIBRATING;
  }

  @Nullable
  @Override
  public String getDeviceName()
  {
    return null;
  }

  @Override
  public void onSpeed(int speedKmh, long elapsedRealtimeMs)
  {
    mSpeedKmh = speedKmh;
    mSpeedTimeMs = elapsedRealtimeMs;
    if (speedKmh > 0)
      mStoppedSinceMs = 0;
    else if (mStoppedSinceMs == 0)
      mStoppedSinceMs = elapsedRealtimeMs;
    mListener.onSpeed(speedKmh, elapsedRealtimeMs);
  }

  @Override
  public void onStateChanged(@NonNull Elm327Client.State state)
  {
    mState = switch (state)
    {
      case DISCONNECTED -> State.DISCONNECTED;
      case CONNECTING -> State.CONNECTING;
      case NO_ADAPTER -> State.NO_ADAPTER;
      case NO_CAR_DATA -> State.NO_CAR_DATA;
      case CONNECTED -> State.CONNECTED;
    };
  }

  @Override
  public void onSensorChanged(SensorEvent event)
  {
    if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
    {
      System.arraycopy(event.values, 0, mAccel, 0, 3);
      mHasAccel = true;
      return;
    }

    final float[] gyro = {event.values[0], event.values[1], event.values[2]};
    final double dt = mLastGyroTimestampNs == 0 ? 0 : (event.timestamp - mLastGyroTimestampNs) / 1e9;
    mLastGyroTimestampNs = event.timestamp;
    if (dt <= 0 || dt > MAX_SENSOR_DT_SEC || !mHasAccel)
      return;

    updateCalibration(gyro);
    final double yawDeltaDeg = isCalibrated() ? GyroCalibrator.yawRateDeg(gyro, mBias, mUp) * dt : 0;
    mListener.onMotion(yawDeltaDeg, dt, event.timestamp);
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {}

  private boolean isCarMoving(long now)
  {
    return mSpeedKmh > 0 && now - mSpeedTimeMs < SPEED_STALE_MS;
  }

  private void updateCalibration(@NonNull float[] gyro)
  {
    final long now = SystemClock.elapsedRealtime();
    if (mCalibrationState == InertialNavigator.CalibrationState.CALIBRATING)
    {
      mCalibrator.add(gyro, mAccel);
      // A car driving straight is as still for the phone as a standing one.
      if (isCarMoving(now))
        mCalibrationMoved = true;
      if (mCalibrator.isComplete())
      {
        if (mCalibrator.isStill() && !mCalibrationMoved)
        {
          applyCalibration(mCalibrator);
          mCalibrationState = InertialNavigator.CalibrationState.DONE;
        }
        else
        {
          mCalibrationState = InertialNavigator.CalibrationState.FAILED_MOVING;
        }
        Logger.i(TAG, "Calibration: " + mCalibrationState);
      }
      return;
    }

    // Refresh the gyroscope bias on every stop, it changes with the temperature.
    final boolean stopped = mSpeedKmh == 0 && now - mSpeedTimeMs < SPEED_STALE_MS && mStoppedSinceMs != 0
                         && now - mStoppedSinceMs > AUTO_CALIBRATION_DELAY_MS;
    if (!stopped)
    {
      mAutoCalibrator.reset();
      mPreviousAutoBias = null;
      return;
    }
    mAutoCalibrator.add(gyro, mAccel);
    if (mAutoCalibrator.isComplete())
    {
      if (mAutoCalibrator.isStill())
        applyAutoCalibration(now - mStoppedSinceMs);
      else
        mPreviousAutoBias = null;
      mAutoCalibrator.reset();
    }
  }

  private void applyAutoCalibration(long stoppedMs)
  {
    final float[] up = mAutoCalibrator.getUp();
    final float[] previous = mPreviousAutoBias;
    mPreviousAutoBias = mAutoCalibrator.getBias();
    if (previous == null)
      return;
    final float[] bias = GyroCalibrator.confirm(previous, mPreviousAutoBias, up);
    if (bias == null)
      return;
    final double changeDeg = mBias != null ? GyroCalibrator.yawRateDeg(bias, mBias, up) : 0;
    if (!GyroCalibrator.isBiasChangeAllowed(mBias, bias, up, stoppedMs))
    {
      Logger.i(TAG, "Automatic calibration ignored: the car turns slowly at " + changeDeg + " deg/s");
      return;
    }
    if (Math.abs(changeDeg) > 0.1)
      Logger.i(TAG, "Automatic calibration changes the yaw by " + changeDeg + " deg/s");
    mBias = bias;
    mUp = up;
    mCalibrationState = InertialNavigator.CalibrationState.DONE;
    Logger.d(TAG, "Automatic calibration at a stop");
  }

  private void applyCalibration(@NonNull GyroCalibrator calibrator)
  {
    mBias = calibrator.getBias();
    mUp = calibrator.getUp();
  }
}
