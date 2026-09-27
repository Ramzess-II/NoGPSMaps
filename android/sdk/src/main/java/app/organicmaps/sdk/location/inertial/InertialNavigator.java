package app.organicmaps.sdk.location.inertial;

import android.content.Context;
import android.location.Location;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.util.Config;
import app.organicmaps.sdk.util.log.Logger;

/**
 * Inertial navigation: the car speed and turns come from a {@link MotionSource}, the phone fixed on the
 * dashboard with an ELM327 OBD-II adapter or the ESP32 sensor box.
 * <p>
 * It needs a reference position and heading (from GPS while it was trusted, manual marks or the user) and
 * a calibrated gyroscope.
 */
public class InertialNavigator implements MotionSource.Listener
{
  private static final String TAG = InertialNavigator.class.getSimpleName();

  public static final String PROVIDER = "inertial";

  // Without fresh speed the position can't be calculated.
  private static final long SPEED_STALE_MS = 2000;
  private static final long OUTPUT_INTERVAL_MS = 200;
  private static final double MOVING_SPEED_MPS = 0.5;
  // GPS bearing is noisy at low speeds.
  private static final double MIN_GPS_HEADING_SPEED_MPS = 3;
  // A longer pause of the source is a lost connection, the car could do anything meanwhile.
  private static final double MAX_MOTION_DT_SEC = 1;
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
  // The pause is over when the car drives this fast for a while: nobody drives so fast backwards.
  private static final int AUTO_RESUME_SPEED_KMH = 15;
  private static final long AUTO_RESUME_MS = 3000;

  public enum CalibrationState
  {
    NONE,
    CALIBRATING,
    DONE,
    // The phone or the car moved during the calibration.
    FAILED_MOVING,
    // The sensor box has been moved in the car since its calibration.
    MOUNT_MOVED,
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
  private final Context mContext;
  @NonNull
  private final Listener mListener;
  @NonNull
  private final Roads mRoads;
  private long mLastSnapMs;
  private boolean mOnRoad;
  private double mLastSnapShiftM = Double.NaN;
  @Nullable
  private MotionSource mSource;
  private boolean mStarted;

  private final DeadReckoning mDeadReckoning = new DeadReckoning();
  @NonNull
  private HeadingSource mHeadingSource = HeadingSource.NONE;

  private long mLastOutputMs;

  private int mSpeedKmh = -1;
  private long mSpeedTimeMs;

  private final SpeedScale mSpeedScale = new SpeedScale();

  // The turn the car is making now: where it has started and where the car looked before it.
  private boolean mTurning;
  private double mTurnFromBearing;
  private double mTurnStartLat;
  private double mTurnStartLon;
  private long mTurnCalmSinceNs;
  // How far the last turn has moved the car to its crossing, for the log of a drive.
  private double mLastTurnShiftM = Double.NaN;

  // While the car maneuvers, e.g. parks or turns around in several moves, the speed from the car is always
  // positive and would move the car forward. The car stays where it is then, the gyroscope still follows
  // its rotation.
  private boolean mPaused;
  // Elapsed realtime since the car drives fast enough to end the pause, 0 if it doesn't.
  private long mFastSinceMs;

  /**
   * @param roads the roads and the followed route, the car is kept on them.
   */
  public InertialNavigator(@NonNull Context context, @NonNull Listener listener, @NonNull Roads roads)
  {
    mContext = context;
    mListener = listener;
    mRoads = roads;
  }

  /**
   * Starts the motion source chosen in the settings.
   */
  public void start()
  {
    if (mStarted)
      stop();
    mStarted = true;
    // The speedometer error is the same on every trip.
    mSpeedScale.set(Config.getNoGpsSpeedScale());
    mSource = Config.isNoGpsEsp32Source()
                ? new Esp32MotionSource(mContext, Config.getNoGpsEsp32Address(), this)
                : new PhoneMotionSource(mContext, Config.getElm327Address(), this);
    mSource.start();
  }

  public void stop()
  {
    if (!mStarted)
      return;
    mStarted = false;
    if (mSource != null)
      mSource.stop();
    mSource = null;
    mSpeedKmh = -1;
  }

  /**
   * Starts the gyroscope calibration. The gyroscope must be fixed and the car must stand still for ~3 s.
   */
  public void calibrate()
  {
    Logger.i(TAG);
    if (mSource != null)
      mSource.calibrate();
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
    if (mDeadReckoning.hasHeading())
      return;
    // Without GPS and marks the car looks along its road either way, the user turns it around if it is wrong.
    final double[] road = mRoads.snap(lat, lon, Double.NaN, MAX_SNAP_RADIUS_M);
    if (road != null)
      setHeading(road[2], HeadingSource.ROAD);
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
    return mStarted && mSource != null && mSource.isCalibrated() && isSpeedFresh() && mDeadReckoning.isReady();
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
  public MotionSource.State getSourceState()
  {
    return mSource != null ? mSource.getState() : MotionSource.State.DISCONNECTED;
  }

  /**
   * @return the name of the sensor device to show, null for the phone.
   */
  @Nullable
  public String getDeviceName()
  {
    return mSource != null ? mSource.getDeviceName() : null;
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
    return mSource != null ? mSource.getCalibrationState() : CalibrationState.NONE;
  }

  public int getCalibrationProgressPercent()
  {
    return mSource != null ? mSource.getCalibrationProgressPercent() : 0;
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

  public boolean isPaused()
  {
    return mPaused;
  }

  /**
   * Pauses the movement of the car while it maneuvers, or resumes it: the car is put on the road where it
   * stands, looking the way it has turned to during the pause.
   */
  public void setPaused(boolean paused)
  {
    if (mPaused == paused)
      return;
    Logger.i(TAG, "paused = " + paused);
    mPaused = paused;
    mFastSinceMs = 0;
    mTurning = false;
    if (paused || !mDeadReckoning.isReady())
      return;

    final double heading = mDeadReckoning.getHeading();
    final double[] road = mRoads.snap(mDeadReckoning.getLat(), mDeadReckoning.getLon(), heading, MAX_SNAP_RADIUS_M);
    if (road != null)
      setRoadPosition(road[0], road[1], RoadWalker.orient(road[2], heading));
  }

  @Override
  public void onSpeed(int speedKmh, long elapsedRealtimeMs)
  {
    mSpeedKmh = speedKmh;
    mSpeedTimeMs = elapsedRealtimeMs;
    if (mPaused)
    {
      if (speedKmh < AUTO_RESUME_SPEED_KMH)
        mFastSinceMs = 0;
      else if (mFastSinceMs == 0)
        mFastSinceMs = elapsedRealtimeMs;
      else if (elapsedRealtimeMs - mFastSinceMs >= AUTO_RESUME_MS)
        setPaused(false);
    }
  }

  @Override
  public void onMotion(double yawDeltaDeg, double dt, long timestampNs)
  {
    if (dt <= 0 || dt > MAX_MOTION_DT_SEC)
      return;

    final boolean speedFresh = isSpeedFresh();
    mDeadReckoning.setSpeed(speedFresh ? mSpeedKmh / 3.6 * mSpeedScale.get() : 0);
    // A standing car can't turn, so the remaining gyroscope drift doesn't rotate the heading at stops.
    if (mSource != null && mSource.isCalibrated() && speedFresh && mDeadReckoning.getSpeed() > MOVING_SPEED_MPS)
    {
      if (!mPaused)
        trackTurn(yawDeltaDeg / dt, timestampNs);
      mDeadReckoning.rotate(yawDeltaDeg);
    }
    if (speedFresh && !mPaused)
    {
      mDeadReckoning.advance(dt);
      mSpeedScale.onDistance(mDeadReckoning.getSpeed() * dt);
    }

    final long now = SystemClock.elapsedRealtime();
    if (isReady() && !mPaused && mDeadReckoning.getSpeed() > MOVING_SPEED_MPS
        && now - mLastSnapMs >= SNAP_INTERVAL_MS)
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

}
