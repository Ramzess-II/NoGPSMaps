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
import app.organicmaps.sdk.util.Config;
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
  // A turn at a crossing rotates the car faster than a bend of a road.
  private static final double TURN_START_RATE_DEG = 10;
  // The turn is over when the car goes straight for a while.
  private static final double TURN_END_RATE_DEG = 4;
  private static final long TURN_END_CALM_NS = 1_000_000_000L;
  // The car starts turning a few meters before the middle of the crossing.
  private static final double TURN_CORNER_AHEAD_M = 5;
  // The car is moved to the new road found this close to the moved position.
  private static final double TURN_SNAP_RADIUS_M = 20;

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
    // The direction of the road the car is on, the user has chosen one of its two ways.
    ROAD,
  }

  public interface Listener
  {
    void onInertialLocation(@NonNull Location location);
  }

  @NonNull
  private final SensorManager mSensorManager;
  @NonNull
  private final Listener mListener;
  @NonNull
  private final Roads mRoads;
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

  // The turn the car is making now: where it has started and where the car looked before it.
  private boolean mTurning;
  private double mTurnFromBearing;
  private double mTurnStartLat;
  private double mTurnStartLon;
  private long mTurnCalmSinceNs;
  // How far the last turn has moved the car to its crossing, for the log of a drive.
  private double mLastTurnShiftM = Double.NaN;

  /**
   * @param roads the roads and the followed route, the car is kept on them.
   */
  public InertialNavigator(@NonNull Context context, @NonNull Listener listener, @NonNull Roads roads)
  {
    mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
    mListener = listener;
    mRoads = roads;
  }

  /**
   * @param elm327Address Bluetooth address of the ELM327 adapter, null if it is not chosen yet.
   */
  public void start(@Nullable String elm327Address)
  {
    if (mStarted)
      stop();
    mStarted = true;
    // The speedometer error is the same on every trip.
    mSpeedScale.set(Config.getNoGpsSpeedScale());
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
   * A trusted GPS position. The inertial navigation continues from it.
   */
  public void onGpsPosition(@NonNull Location location)
  {
    mDeadReckoning.setPosition(location.getLatitude(), location.getLongitude());
    mTurning = false;
    if (location.hasBearing() && location.hasSpeed() && location.getSpeed() >= MIN_GPS_HEADING_SPEED_MPS)
      setHeading(location.getBearing(), HeadingSource.GPS);
  }

  /**
   * The position is on a road: its direction is more reliable than the calculated one.
   * @param bearingDeg direction of the road in the direction of the movement.
   */
  public void setRoadPosition(double lat, double lon, double bearingDeg)
  {
    mDeadReckoning.setPosition(lat, lon);
    mTurning = false;
    setHeading(bearingDeg, HeadingSource.ROAD);
  }

  /**
   * The user has corrected the lag of the calculated position along the road.
   * @param appliedM the distance the position was moved by, negative if it was moved back.
   */
  public void onPositionCorrected(double lat, double lon, double bearingDeg, double appliedM)
  {
    mSpeedScale.onCorrection(appliedM);
    Config.setNoGpsSpeedScale((float) mSpeedScale.get());
    Logger.i(TAG, "Corrected by " + Math.round(appliedM) + " m, speed scale = " + mSpeedScale.get());
    setRoadPosition(lat, lon, bearingDeg);
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
   * Turns the car around on its road: the user has chosen the other way of the road.
   */
  public void reverseHeading()
  {
    if (mDeadReckoning.hasHeading())
      setHeading(mDeadReckoning.getHeading() + 180, HeadingSource.ROAD);
    mTurning = false;
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

  /**
   * @return how far the last turn has moved the car to its crossing, NaN if no turn was matched yet.
   */
  public double getLastTurnShiftM()
  {
    return mLastTurnShiftM;
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
    {
      final double yawRateDeg = GyroCalibrator.yawRateDeg(gyro, mBias, mUp);
      trackTurn(yawRateDeg, event.timestamp);
      mDeadReckoning.rotate(yawRateDeg * dt);
    }
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
    final double[] snapped = mRoads.snap(lat, lon, mDeadReckoning.getHeading(), radius);
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
   * Follows a turn of the car. When the turn is over, the car is moved to the crossing it has turned at.
   * @param yawRateDeg clockwise rotation speed of the car.
   */
  private void trackTurn(double yawRateDeg, long timestampNs)
  {
    if (!isReady())
      return;
    if (!mTurning)
    {
      if (Math.abs(yawRateDeg) < TURN_START_RATE_DEG)
        return;
      mTurning = true;
      mTurnFromBearing = mDeadReckoning.getHeading();
      mTurnStartLat = mDeadReckoning.getLat();
      mTurnStartLon = mDeadReckoning.getLon();
      mTurnCalmSinceNs = 0;
      return;
    }

    if (Math.abs(yawRateDeg) >= TURN_END_RATE_DEG)
    {
      mTurnCalmSinceNs = 0;
      return;
    }
    if (mTurnCalmSinceNs == 0)
    {
      mTurnCalmSinceNs = timestampNs;
      return;
    }
    if (timestampNs - mTurnCalmSinceNs < TURN_END_CALM_NS)
      return;

    mTurning = false;
    matchTurn();
  }

  private void matchTurn()
  {
    final double toBearing = mDeadReckoning.getHeading();
    final double[] corner = DeadReckoning.move(mTurnStartLat, mTurnStartLon, mTurnFromBearing, TURN_CORNER_AHEAD_M);
    final double[] crossing = TurnMatcher.findCrossing(mRoads, corner[0], corner[1], mTurnFromBearing, toBearing,
                                                       mDeadReckoning.getAccuracy());
    if (crossing == null)
    {
      Logger.i(TAG, "Turn from " + Math.round(mTurnFromBearing) + " to " + Math.round(toBearing)
                        + " is not at a crossing");
      return;
    }

    // The whole turn is moved to the crossing: the car has driven the same way after it.
    final double lat = mDeadReckoning.getLat() + crossing[0] - corner[0];
    final double lon = mDeadReckoning.getLon() + crossing[1] - corner[1];
    final double[] road = mRoads.snap(lat, lon, toBearing, TURN_SNAP_RADIUS_M);
    if (road == null)
    {
      Logger.i(TAG, "No road after the turn at the crossing " + crossing[0] + "," + crossing[1]);
      return;
    }
    mLastTurnShiftM = RoadWalker.distance(corner[0], corner[1], crossing[0], crossing[1]);
    Logger.i(TAG, "Turn from " + Math.round(mTurnFromBearing) + " to " + Math.round(toBearing)
                      + " is moved to the crossing by " + Math.round(mLastTurnShiftM) + " m");
    // The crossing is a known place, the distance error is gone.
    setRoadPosition(road[0], road[1], RoadWalker.orient(road[2], toBearing));
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
