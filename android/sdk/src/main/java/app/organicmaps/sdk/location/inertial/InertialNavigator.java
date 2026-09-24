package app.organicmaps.sdk.location.inertial;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.util.log.Logger;

/**
 * Inertial navigation with the phone fixed on the dashboard: the car speed comes from an ELM327 OBD-II adapter,
 * turns come from the phone gyroscope.
 * <p>
 * It needs a reference position and heading (from GPS while it was trusted, manual marks or the user) and
 * a calibrated gyroscope. The gyroscope is calibrated by the user and automatically on every stop.
 */
public class InertialNavigator implements SensorEventListener, Elm327Client.Listener
{
  private static final String TAG = InertialNavigator.class.getSimpleName();

  public static final String PROVIDER = "inertial";

  // Without fresh speed the position can't be calculated.
  private static final long SPEED_STALE_MS = 2000;
  private static final long OUTPUT_INTERVAL_MS = 200;
  // Wait after the car stops before the automatic calibration, the car body still sways.
  private static final long AUTO_CALIBRATION_DELAY_MS = 2000;
  private static final double MOVING_SPEED_MPS = 0.5;
  // GPS bearing is noisy at low speeds.
  private static final double MIN_GPS_HEADING_SPEED_MPS = 3;
  private static final double MAX_SENSOR_DT_SEC = 0.1;
  private static final long SNAP_INTERVAL_MS = 1000;
  // Don't look for roads too far: a wrong road is worse than none.
  private static final double MIN_SNAP_RADIUS_M = 20;
  private static final double MAX_SNAP_RADIUS_M = 60;

  public enum CalibrationState
  {
    NONE,
    CALIBRATING,
    DONE,
    // The phone or the car moved during the calibration.
    FAILED_MOVING,
  }

  public enum HeadingSource
  {
    NONE,
    GPS,
    MANUAL_MARKS,
    // The direction of the road the user has marked the position on.
    ROAD,
    USER,
  }

  public interface Listener
  {
    void onInertialLocation(@NonNull Location location);
  }

  public interface RoadSnapper
  {
    /**
     * @return {latitude, longitude, road bearing} of the closest point on the route or a road, or null.
     */
    @Nullable
    double[] snap(double lat, double lon, double bearing, double radius);
  }

  @NonNull
  private final SensorManager mSensorManager;
  @NonNull
  private final Listener mListener;
  @NonNull
  private final RoadSnapper mRoadSnapper;
  private long mLastSnapMs;
  private boolean mOnRoad;
  private double mLastSnapShiftM = Double.NaN;
  @Nullable
  private Elm327Client mElm327;
  @NonNull
  private Elm327Client.State mElm327State = Elm327Client.State.DISCONNECTED;
  private boolean mStarted;

  private final DeadReckoning mDeadReckoning = new DeadReckoning();
  @NonNull
  private HeadingSource mHeadingSource = HeadingSource.NONE;

  private final GyroCalibrator mCalibrator = new GyroCalibrator();
  private final GyroCalibrator mAutoCalibrator = new GyroCalibrator();
  @NonNull
  private CalibrationState mCalibrationState = CalibrationState.NONE;
  @Nullable
  private float[] mBias;
  @Nullable
  private float[] mUp;

  private final float[] mAccel = new float[3];
  private boolean mHasAccel;
  private long mLastGyroTimestampNs;
  private long mLastOutputMs;

  private int mSpeedKmh = -1;
  private long mSpeedTimeMs;
  private long mStoppedSinceMs;

  private final SpeedScale mSpeedScale = new SpeedScale();

  public InertialNavigator(@NonNull Context context, @NonNull Listener listener, @NonNull RoadSnapper roadSnapper)
  {
    mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
    mListener = listener;
    mRoadSnapper = roadSnapper;
  }

  /**
   * @param elm327Address Bluetooth address of the ELM327 adapter, null if it is not chosen yet.
   */
  public void start(@Nullable String elm327Address)
  {
    if (mStarted)
      stop();
    mStarted = true;
    Logger.i(TAG, "ELM327 = " + elm327Address);

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

    if (elm327Address != null)
    {
      mElm327 = new Elm327Client(elm327Address, this);
      mElm327.start();
    }
  }

  public void stop()
  {
    if (!mStarted)
      return;
    mStarted = false;
    mSensorManager.unregisterListener(this);
    if (mElm327 != null)
      mElm327.stop();
    mElm327 = null;
    mElm327State = Elm327Client.State.DISCONNECTED;
    mSpeedKmh = -1;
    mLastGyroTimestampNs = 0;
    // The speed scale is valid for the current trip only: the load and the tyre pressure change.
    mSpeedScale.reset();
  }

  /**
   * Starts the gyroscope calibration. The phone must be fixed and the car must stand still for ~3 s.
   */
  public void calibrate()
  {
    Logger.i(TAG);
    mCalibrator.reset();
    mCalibrationState = CalibrationState.CALIBRATING;
  }

  /**
   * A trusted position: GPS or a manual mark. The inertial navigation continues from it.
   */
  public void onReferencePosition(@NonNull Location location, boolean isGps)
  {
    mDeadReckoning.setPosition(location.getLatitude(), location.getLongitude());
    if (isGps && location.hasBearing() && location.hasSpeed() && location.getSpeed() >= MIN_GPS_HEADING_SPEED_MPS)
      setHeading(location.getBearing(), HeadingSource.GPS);
    // The direction set by the user or taken from a road is more reliable than the one from rough marks.
    else if (!isGps && location.hasBearing() && !isHeadingReliable())
      setHeading(location.getBearing(), HeadingSource.MANUAL_MARKS);
  }

  /**
   * The position is on a road: its direction is more reliable than the calculated one.
   * @param bearingDeg direction of the road in the direction of the movement.
   */
  public void setRoadPosition(double lat, double lon, double bearingDeg)
  {
    mDeadReckoning.setPosition(lat, lon);
    setHeading(bearingDeg, HeadingSource.ROAD);
  }

  /**
   * The user has corrected the lag of the calculated position along the road.
   * @param appliedM the distance the position was moved by, negative if it was moved back.
   */
  public void onPositionCorrected(double lat, double lon, double bearingDeg, double appliedM)
  {
    mSpeedScale.onCorrection(appliedM);
    Logger.i(TAG, "Corrected by " + Math.round(appliedM) + " m, speed scale = " + mSpeedScale.get());
    setRoadPosition(lat, lon, bearingDeg);
  }

  private boolean isHeadingReliable()
  {
    return mHeadingSource == HeadingSource.USER || mHeadingSource == HeadingSource.ROAD;
  }

  /**
   * @return the ratio the speed from the car is multiplied by, 1 if it is not corrected yet.
   */
  public double getSpeedScale()
  {
    return mSpeedScale.get();
  }

  /**
   * Moves the position without changing the heading, e.g. to the shown position while the inertial
   * navigation doesn't work yet.
   */
  public void setPosition(double lat, double lon)
  {
    mDeadReckoning.setPosition(lat, lon);
  }

  /**
   * @param headingDeg the direction the car looks at, clockwise from the north.
   */
  public void setHeading(double headingDeg, @NonNull HeadingSource source)
  {
    mDeadReckoning.setHeading(headingDeg);
    mHeadingSource = source;
  }

  /**
   * Turns the car direction, e.g. when the user adjusts it.
   * @param deltaDeg clockwise rotation.
   */
  public void rotateHeading(double deltaDeg)
  {
    if (mDeadReckoning.hasHeading())
      setHeading(mDeadReckoning.getHeading() + deltaDeg, HeadingSource.USER);
  }

  /**
   * @return the car direction, clockwise from the north, or NaN if it is unknown.
   */
  public double getHeading()
  {
    return mDeadReckoning.hasHeading() ? mDeadReckoning.getHeading() : Double.NaN;
  }

  public boolean isReady()
  {
    return mStarted && mBias != null && isSpeedFresh() && mDeadReckoning.isReady();
  }

  @Nullable
  public Location getLocation()
  {
    if (!mDeadReckoning.hasPosition())
      return null;
    final Location location = new Location(PROVIDER);
    location.setLatitude(mDeadReckoning.getLat());
    location.setLongitude(mDeadReckoning.getLon());
    location.setAccuracy((float) mDeadReckoning.getAccuracy());
    if (mDeadReckoning.hasHeading())
      location.setBearing((float) mDeadReckoning.getHeading());
    location.setSpeed((float) mDeadReckoning.getSpeed());
    location.setTime(System.currentTimeMillis());
    location.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
    return location;
  }

  @NonNull
  public Elm327Client.State getElm327State()
  {
    return mElm327State;
  }

  /**
   * @return the last speed in km/h, or -1 if it is unknown or stale.
   */
  public int getSpeedKmh()
  {
    return isSpeedFresh() ? mSpeedKmh : -1;
  }

  @NonNull
  public CalibrationState getCalibrationState()
  {
    return mCalibrationState;
  }

  public int getCalibrationProgressPercent()
  {
    return mCalibrator.getProgressPercent();
  }

  @NonNull
  public HeadingSource getHeadingSource()
  {
    return mHeadingSource;
  }

  public boolean hasPosition()
  {
    return mDeadReckoning.hasPosition();
  }

  public double getDistanceSinceFix()
  {
    return mDeadReckoning.getDistanceSinceFix();
  }

  private boolean isSpeedFresh()
  {
    return mSpeedKmh >= 0 && SystemClock.elapsedRealtime() - mSpeedTimeMs < SPEED_STALE_MS;
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
  }

  @Override
  public void onStateChanged(@NonNull Elm327Client.State state)
  {
    mElm327State = state;
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

    final boolean speedFresh = isSpeedFresh();
    mDeadReckoning.setSpeed(speedFresh ? mSpeedKmh / 3.6 * mSpeedScale.get() : 0);
    // A standing car can't turn, so the remaining gyroscope drift doesn't rotate the heading at stops.
    if (mBias != null && mUp != null && speedFresh && mDeadReckoning.getSpeed() > MOVING_SPEED_MPS)
      mDeadReckoning.rotate(GyroCalibrator.yawRateDeg(gyro, mBias, mUp) * dt);
    if (speedFresh)
    {
      mDeadReckoning.advance(dt);
      mSpeedScale.onDistance(mDeadReckoning.getSpeed() * dt);
    }

    final long now = SystemClock.elapsedRealtime();
    if (isReady() && mDeadReckoning.getSpeed() > MOVING_SPEED_MPS && now - mLastSnapMs >= SNAP_INTERVAL_MS)
    {
      mLastSnapMs = now;
      snapToRoad();
    }
    if (isReady() && now - mLastOutputMs >= OUTPUT_INTERVAL_MS)
    {
      mLastOutputMs = now;
      final Location location = getLocation();
      if (location != null)
        mListener.onInertialLocation(location);
    }
  }

  private void snapToRoad()
  {
    final double radius =
        Math.min(MAX_SNAP_RADIUS_M, Math.max(MIN_SNAP_RADIUS_M, mDeadReckoning.getAccuracy()));
    final double lat = mDeadReckoning.getLat();
    final double lon = mDeadReckoning.getLon();
    final double[] snapped = mRoadSnapper.snap(lat, lon, mDeadReckoning.getHeading(), radius);
    if (snapped == null)
    {
      mOnRoad = false;
      mLastSnapShiftM = Double.NaN;
      return;
    }
    // Roads are searched in a square, its corners are farther than the radius.
    final float[] shift = new float[1];
    Location.distanceBetween(lat, lon, snapped[0], snapped[1], shift);
    mLastSnapShiftM = shift[0];
    mOnRoad = shift[0] <= radius && mDeadReckoning.snapToRoad(snapped[0], snapped[1], snapped[2]);
  }

  /**
   * @return true if the last position was snapped to a road or the route.
   */
  public boolean isOnRoad()
  {
    return mOnRoad;
  }

  /**
   * @return how far the closest road was from the calculated position, NaN if there was no road.
   */
  public double getLastSnapShiftM()
  {
    return mLastSnapShiftM;
  }

  private void updateCalibration(@NonNull float[] gyro)
  {
    if (mCalibrationState == CalibrationState.CALIBRATING)
    {
      mCalibrator.add(gyro, mAccel);
      if (mCalibrator.isComplete())
      {
        if (mCalibrator.isStill())
        {
          applyCalibration(mCalibrator);
          mCalibrationState = CalibrationState.DONE;
        }
        else
        {
          mCalibrationState = CalibrationState.FAILED_MOVING;
        }
        Logger.i(TAG, "Calibration: " + mCalibrationState);
      }
      return;
    }

    // Refresh the gyroscope bias on every stop, it changes with the temperature.
    final boolean stopped = isSpeedFresh() && mSpeedKmh == 0 && mStoppedSinceMs != 0
                         && SystemClock.elapsedRealtime() - mStoppedSinceMs > AUTO_CALIBRATION_DELAY_MS;
    if (!stopped)
    {
      mAutoCalibrator.reset();
      return;
    }
    mAutoCalibrator.add(gyro, mAccel);
    if (mAutoCalibrator.isComplete())
    {
      if (mAutoCalibrator.isStill())
      {
        applyCalibration(mAutoCalibrator);
        if (mCalibrationState != CalibrationState.DONE)
          mCalibrationState = CalibrationState.DONE;
        Logger.d(TAG, "Automatic calibration at a stop");
      }
      mAutoCalibrator.reset();
    }
  }

  private void applyCalibration(@NonNull GyroCalibrator calibrator)
  {
    mBias = calibrator.getBias();
    mUp = calibrator.getUp();
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}
