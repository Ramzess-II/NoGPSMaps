package app.organicmaps.sdk.location;

import static android.Manifest.permission.ACCESS_COARSE_LOCATION;
import static android.Manifest.permission.ACCESS_FINE_LOCATION;

import android.annotation.SuppressLint;
import android.app.PendingIntent;
import android.content.Context;
import android.location.Location;
import android.location.LocationManager;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresPermission;
import androidx.annotation.UiThread;
import androidx.core.content.ContextCompat;
import androidx.core.location.GnssStatusCompat;
import androidx.core.location.LocationListenerCompat;
import androidx.core.location.LocationManagerCompat;
import androidx.core.location.LocationRequestCompat;
import app.organicmaps.sdk.Framework;
import app.organicmaps.sdk.Map;
import app.organicmaps.sdk.bookmarks.data.MapObject;
import app.organicmaps.sdk.location.inertial.DeadReckoning;
import app.organicmaps.sdk.location.inertial.InertialNavigator;
import app.organicmaps.sdk.location.inertial.RoadWalker;
import app.organicmaps.sdk.location.inertial.Roads;
import app.organicmaps.sdk.routing.JunctionInfo;
import app.organicmaps.sdk.routing.RoutingController;
import app.organicmaps.sdk.util.Config;
import app.organicmaps.sdk.util.NetworkPolicy;
import app.organicmaps.sdk.util.log.Logger;
import java.util.Locale;
import org.chromium.base.ObserverList;

public class LocationHelper implements BaseLocationProvider.Listener
{
  // Replaced 0 with 100ms. Seems like it is not working on Lineage, EOS, MicroG, ...
  // https://github.com/organicmaps/organicmaps/issues/11076
  // Probably https://github.com/organicmaps/organicmaps/issues/10133
  // Probably https://github.com/organicmaps/organicmaps/issues/9018
  private static final long INTERVAL_FOLLOW_MS = 100;
  private static final long INTERVAL_NOT_FOLLOW_MS = 3000;
  private static final long INTERVAL_NAVIGATION_MS = 100;
  private static final long INTERVAL_TRACK_RECORDING = 100;

  private static final long AGPS_EXPIRATION_TIME_MS = 16 * 60 * 60 * 1000; // 16 hours
  private static final long LOCATION_UPDATE_TIMEOUT_MS = 30 * 1000; // 30 seconds

  public static final String MANUAL_PROVIDER = "manual";
  // The manual position is re-sent to the core periodically, otherwise the location is treated as lost.
  private static final long MANUAL_REPEAT_INTERVAL_MS = 5000;
  // Taps on the map are not precise, so the core matches a manual position to the route within this radius.
  private static final float MANUAL_ACCURACY_M = 50;
  // The car stands on a road, not between houses, but a farther road is not the road the user means.
  private static final double MANUAL_SNAP_RADIUS_M = 30;
  // A road going another way is a crossing road: the car is on it, but it does not drive along it.
  private static final double MAX_ROAD_BEARING_DIFF_DEG = 45;
  // A point of the route closer than this to a road is on it, and goes along it within the angle.
  private static final double ROAD_CHECK_RADIUS_M = 3;
  private static final double MAX_ALONG_ROAD_DIFF_DEG = 30;
  // Closer to the previous position, the side the car has turned to is not known.
  private static final float MIN_TURN_DISTANCE_M = 5;
  // The position stopped at a turn is not moved further: the car can turn to another street there.
  // It is moved again after the car has really left the turn by this distance.
  private static final float SHIFT_BLOCK_RADIUS_M = 10;
  private static final double MIN_SHIFT_M = 0.5;

  // Network (cell towers and Wi-Fi) positions are used to detect spoofed GPS.
  private static final long INTERVAL_NETWORK_MS = 5000;

  // A position without updates for longer than this is shown as lost.
  private static final long POSITION_STALE_MS = 30 * 1000;
  // The fused provider mixes GPS with cell towers and Wi-Fi: less accurate positions are from the network.
  private static final float FUSED_SATELLITE_MAX_ACCURACY_M = 30;
  // A position closer to the route than this is considered on the route even if it is very accurate.
  private static final float MIN_OFF_ROUTE_DISTANCE_M = 50;
  // A position set by the user lies on the axis of the route if the car is on the route. The core keeps the
  // car on the route ahead of a farther one: it moves along the route forward only, and the car would be
  // shown ahead of the mark, e.g. after a mark behind the previous one. The route is rebuilt from it then.
  private static final float MANUAL_OFF_ROUTE_DISTANCE_M = 10;
  // Trusted GPS positions are preferred over the inertial ones while they are this fresh. GPS gives a
  // position every second, so it is lost after a missed one.
  private static final long GPS_FRESH_MS = 2000;
  // Without the inertial navigation a longer pause of GPS is waited for while navigating: the manual mode
  // needs a mark from the user.
  private static final long GPS_LOST_MS = 5000;
  // The first GPS position after the start takes longer: the satellites are searched.
  private static final long GPS_FIRST_FIX_MS = 15_000;
  // GPS farther than this from the inertial position following it is spoofed or broken: the inertial
  // position is calculated for a second only since the previous GPS position.
  private static final double MAX_GPS_INERTIAL_DIFF_M = 30;
  // A less accurate GPS position is jammed, the inertial navigation is better.
  private static final float GPS_MAX_ACCURACY_M = 30;
  // GPS farther from the roads than this plus its accuracy is led away by a spoofer, after several positions
  // in a row: a single one can be noise.
  private static final double MAX_GPS_OFF_ROAD_M = 20;
  private static final int OFF_ROAD_GPS_COUNT = 3;
  // The car speed errors are a few percent, they are measured by accurate GPS only.
  private static final float SPEED_TABLE_GPS_MAX_ACCURACY_M = 10;
  private static final float OFF_ROAD_MIN_SPEED_MPS = 5;
  // The inertial navigation is considered active while its positions are used this recently.
  private static final long INERTIAL_ACTIVE_MS = 1000;
  // Old positions (e.g. the last known one after a provider restart) tell nothing about the car now.
  private static final long GPS_MAX_AGE_MS = 5000;
  // The state is written to the log every second, so a drive can be analysed afterwards.
  private static final long TRIP_LOG_INTERVAL_MS = 1000;
  // GPS gives a position every second, a longer pause means it is lost.
  private static final long GPS_STATUS_MAX_AGE_MS = 10_000;
  // Cell tower positions come once in several seconds.
  private static final long NETWORK_STATUS_MAX_AGE_MS = 60_000;

  public enum PositionSource
  {
    NONE,
    GPS,
    // Cell towers and Wi-Fi.
    NETWORK,
    MANUAL,
    // Car speed from OBD + phone gyroscope.
    INERTIAL
  }

  public interface ManualModeListener
  {
    void onManualModeChanged(boolean enabled);

    /**
     * The manual mode has been left by itself: GPS is trusted again.
     */
    default void onGpsBack() {}

    /**
     * The manual mode has been turned on by itself: GPS is lost or wrong while driving.
     */
    default void onGpsLost() {}

    /**
     * The inertial navigation has left the roads: the car is stopped on the road, the user should mark it.
     */
    default void onRoadLost() {}
  }

  public interface GpsSpoofingListener
  {
    void onGpsSpoofingChanged(boolean spoofed);
  }

  @NonNull
  private final Context mContext;
  @NonNull
  private final SensorHelper mSensorHelper;

  private static final String TAG = LocationState.LOCATION_TAG;

  private final ObserverList<LocationListener> mListeners = new ObserverList<>();
  private final ObserverList.RewindableIterator<LocationListener> mListenersIterator = mListeners.rewindableIterator();

  @Nullable
  private Location mSavedLocation;
  private MapObject mMyPosition;
  @NonNull
  private final LocationProviderFactory mLocationProviderFactory = new LocationProviderFactory();
  @NonNull
  private BaseLocationProvider mLocationProvider;
  @Nullable
  private BaseLocationProvider mOldLocationProvider;
  private long mInterval;
  private boolean mInFirstRun;
  private boolean mActive;
  private final Handler mHandler;
  private final Runnable mLocationTimeoutRunnable = this::notifyLocationUpdateTimeout;

  // In the manual mode GPS is ignored (it is jammed or spoofed) and the position is set by the user.
  private boolean mManualMode;
  // GPS is ignored as if it were jammed, to test the navigation without GPS where GPS works. It is not
  // kept after a restart, so it is not left on by mistake.
  private boolean mGpsDisabled;
  @Nullable
  private Location mManualLocation;
  private final Runnable mManualRepeatRunnable = this::applyManualLocation;
  private final Runnable mTripLogRunnable = this::logTrip;
  // Elapsed realtime of the last position set by the user.
  private long mManualSetTimeMs;
  // The last GPS position good enough to be used. It is kept in the manual mode too, where GPS is not used,
  // to tell the user that GPS works again.
  @Nullable
  private Location mLastGoodGps;

  // After the manual mode the next real position must replace the manual one, even if it is less accurate.
  private boolean mAcceptNextLocation;
  // The source of the last position passed to the core, used to check the route when the source changes.
  @NonNull
  private PositionSource mLastPositionSource = PositionSource.NONE;

  // The turns the position has stopped at, null if the position can be moved that way.
  @Nullable
  private Location mShiftStopForward;
  @Nullable
  private Location mShiftStopBack;

  @Nullable
  private InertialNavigator mInertial;
  private long mLastTrustedGpsMs;
  // Elapsed realtime when the manual mode was left: GPS was ignored in it, so it is waited for anew.
  private long mManualLeftMs;
  // GPS positions off the roads in a row.
  private int mOffRoadGpsCount;
  // Elapsed realtime of the start of the location updates.
  private long mStartTimeMs;
  private long mLastInertialUsedMs;
  private final ObserverList<ManualModeListener> mManualModeListeners = new ObserverList<>();

  private final GpsSpoofingDetector mSpoofingDetector = new GpsSpoofingDetector();
  private final GpsReturnDetector mGpsReturnDetector = new GpsReturnDetector();
  // Tells when GPS has worked long enough to measure the car speed errors by it.
  private final GpsReturnDetector mSpeedTableGpsDetector = new GpsReturnDetector();
  @Nullable
  private Location mNetworkLocation;
  private final ObserverList<GpsSpoofingListener> mSpoofingListeners = new ObserverList<>();
  // Some providers (e.g. Google fused) don't report network positions separately, so request them explicitly.
  private final LocationListenerCompat mNetworkListener = this::onLocationChanged;

  @NonNull
  private final GnssStatusCompat.Callback mGnssStatusCallback = new GnssStatusCompat.Callback() {
    @Override
    public void onStarted()
    {
      Logger.d(TAG);
    }

    @Override
    public void onStopped()
    {
      Logger.d(TAG);
    }

    @Override
    public void onFirstFix(int ttffMillis)
    {
      Logger.d(TAG, "ttffMillis = " + ttffMillis);
    }

    @Override
    public void onSatelliteStatusChanged(@NonNull GnssStatusCompat status)
    {
      int used = 0;
      boolean fixed = false;
      for (int i = 0; i < status.getSatelliteCount(); i++)
      {
        if (status.usedInFix(i))
        {
          used++;
          fixed = true;
        }
      }
      Logger.d(TAG, "total = " + status.getSatelliteCount() + " used = " + used + " fixed = " + fixed);
    }
  };

  public LocationHelper(@NonNull Context context, @NonNull SensorHelper sensorHelper)
  {
    mContext = context;
    mSensorHelper = sensorHelper;
    mLocationProvider = mLocationProviderFactory.getProvider(mContext, this);
    mHandler = new Handler(Looper.getMainLooper());
  }

  /**
   * @return MapObject.MY_POSITION, null if location is not yet determined or "My position" button is switched off.
   */
  @Nullable
  public MapObject getMyPosition()
  {
    if (!isActive())
    {
      mMyPosition = null;
      return null;
    }

    if (mSavedLocation == null)
      return null;

    if (mMyPosition == null)
      mMyPosition = MapObject.createMapObject(MapObject.MY_POSITION, "", "", mSavedLocation.getLatitude(),
                                              mSavedLocation.getLongitude());

    return mMyPosition;
  }

  /**
   * Obtains last known location.
   * @return {@code null} if no location is saved.
   */
  @Nullable
  public Location getSavedLocation()
  {
    return mSavedLocation;
  }

  /**
   * Indicates about whether a location provider is polling location updates right now or not.
   */
  public boolean isActive()
  {
    return mActive;
  }

  private void notifyLocationUpdated()
  {
    if (mSavedLocation == null)
      throw new IllegalStateException("No saved location");

    mHandler.removeCallbacks(mLocationTimeoutRunnable);
    mHandler.postDelayed(mLocationTimeoutRunnable, LOCATION_UPDATE_TIMEOUT_MS); // Reset the timeout.

    mListenersIterator.rewind();
    while (mListenersIterator.hasNext())
      mListenersIterator.next().onLocationUpdated(mSavedLocation);

    // If we are still in the first run mode, i.e. user is staying on the first run screens,
    // not on the map, we mustn't post location update to the core. Only this preserving allows us
    // to play nice zoom animation once a user will leave first screens and will see a map.
    if (mInFirstRun)
    {
      Logger.d(TAG, "Location update is obtained and must be ignored, because the app is in a first run mode");
      return;
    }

    final LocationCompatExtractor.Altitude altitude = LocationCompatExtractor.getAltitude(mSavedLocation);
    LocationState.nativeLocationUpdated(
        mSavedLocation.getTime(), mSavedLocation.getLatitude(), mSavedLocation.getLongitude(),
        mSavedLocation.getAccuracy(), altitude != null ? altitude.altitude() : 0,
        altitude != null ? altitude.accuracy() : -1, mSavedLocation.hasSpeed() ? mSavedLocation.getSpeed() : -1,
        mSavedLocation.hasBearing() ? mSavedLocation.getBearing() : -1);
    showCarHeading();
  }

  private void notifyLocationUpdateTimeout()
  {
    mHandler.removeCallbacks(mLocationTimeoutRunnable);
    if (!isActive())
    {
      Logger.w(TAG, "Provider is not active");
      return;
    }

    Logger.d(TAG);
    mListenersIterator.rewind();
    while (mListenersIterator.hasNext())
      mListenersIterator.next().onLocationUpdateTimeout();
  }

  @Override
  public void onLocationChanged(@NonNull Location location)
  {
    Logger.d(TAG, "provider = " + mLocationProvider.getClass().getSimpleName() + " location = " + location);

    if (!isActive())
    {
      Logger.w(TAG, "Provider is not active");
      return;
    }

    final boolean isNetwork = LocationManager.NETWORK_PROVIDER.equals(location.getProvider());
    final boolean wasSpoofed = mSpoofingDetector.isSpoofed();
    final long timeMs = location.getElapsedRealtimeNanos() / 1_000_000;
    boolean trusted = true;
    if (isNetwork)
    {
      mNetworkLocation = location;
      mSpoofingDetector.onNetworkPosition(location.getLatitude(), location.getLongitude(), location.getAccuracy(),
                                          timeMs);
    }
    else
    {
      trusted = mSpoofingDetector.checkSatellitePosition(location.getLatitude(), location.getLongitude(), timeMs);
    }

    final boolean spoofed = mSpoofingDetector.isSpoofed();
    if (spoofed != wasSpoofed)
      onGpsSpoofingChanged(spoofed);

    if (trusted && !isNetwork && mInertial != null && isInertialNavigationEnabled() && isSpeedTableGps(location))
      mInertial.onSpeedTableGps(location);

    if (mGpsDisabled && !isNetwork && sourceOf(location) == PositionSource.GPS)
    {
      Logger.d(TAG, "GPS is disabled, ignoring location = " + location);
      return;
    }

    if (!trusted)
    {
      mGpsReturnDetector.reset();
      Logger.w(TAG, "Untrusted location = " + location);
      return;
    }

    final long ageMs = SystemClock.elapsedRealtime() - location.getElapsedRealtimeNanos() / 1_000_000;
    final boolean goodGps = sourceOf(location) == PositionSource.GPS && ageMs <= GPS_MAX_AGE_MS
                         && LocationUtils.isAccuracySatisfied(location)
                         && location.getAccuracy() <= GPS_MAX_ACCURACY_M;
    if (goodGps)
      mLastGoodGps = location;
    if (mManualMode && goodGps && isGpsBack(location))
    {
      Logger.i(TAG, "GPS is trusted again, leaving the manual mode, location = " + location);
      setManualMode(false);
      for (ManualModeListener listener : mManualModeListeners)
        listener.onGpsBack();
    }
    if (!mManualMode && goodGps && isOffRoadWhileDriving(location))
    {
      // The last position on the road is kept, the inertial navigation continues from it.
      if (++mOffRoadGpsCount >= OFF_ROAD_GPS_COUNT)
        enterManualModeByItself("GPS has left the roads, location = " + location);
      else
        Logger.w(TAG, "GPS is off the roads, location = " + location);
      return;
    }
    mOffRoadGpsCount = 0;
    if (!mManualMode && goodGps && contradictsInertial(location))
    {
      enterManualModeByItself("GPS contradicts the inertial position, location = " + location);
      return;
    }
    if (!mManualMode && goodGps)
    {
      mLastTrustedGpsMs = SystemClock.elapsedRealtime();
      if (mInertial != null)
        mInertial.onGpsPosition(location);
    }

    if (mManualMode)
    {
      Logger.d(TAG, "Manual mode is on, ignoring location = " + location);
      return;
    }

    // Cell tower positions jump by hundreds of meters. It is tolerable while the user looks at the map, but
    // while navigating they would throw the car off the route, so only GPS and dead reckoning are used.
    if (isNetwork && mSavedLocation != null && RoutingController.get().isNavigating())
    {
      Logger.d(TAG, "Navigating, ignoring the network location = " + location);
      return;
    }

    if (isNetwork && isInertialActive())
    {
      Logger.d(TAG, "Inertial navigation is more accurate than location = " + location);
      return;
    }

    if (!LocationUtils.isAccuracySatisfied(location))
    {
      Logger.w(TAG, "Unsatisfied accuracy for location = " + location);
      return;
    }

    // While GPS is spoofed network positions are the only source, they must not compete with the spoofed ones.
    if (mSavedLocation != null && !(spoofed && isNetwork) && !mAcceptNextLocation)
    {
      if (!LocationUtils.isLocationBetterThanLast(location, mSavedLocation))
      {
        Logger.d(TAG, "The new " + location + " is worse than the last " + mSavedLocation);
        return;
      }
    }

    mAcceptNextLocation = false;
    mSavedLocation = location;
    mMyPosition = null;

    // While the inertial navigation doesn't work, it starts from the shown position.
    if (mInertial != null && !isInertialActive())
      mInertial.setPosition(location.getLatitude(), location.getLongitude());

    // The core detects that the user left the route only after several moving GPS positions. Check the route at once
    // when the position source changes (e.g. from the manual position, which can be wrong, to GPS or cell towers),
    // and on every network position, as they are rare and often don't change while the user stands still.
    final PositionSource source = sourceOf(location);
    if (source != mLastPositionSource || source == PositionSource.NETWORK)
      rebuildRouteIfOffRoute(location);
    mLastPositionSource = source;

    notifyLocationUpdated();
  }

  /**
   * @return true if GPS has worked long enough in the manual mode and agrees with the position known without
   * it, so the manual mode is not needed anymore.
   */
  private boolean isGpsBack(@NonNull Location gps)
  {
    double ownErrorM = Double.NaN;
    Location own = null;
    if (isInertialActive() && mSavedLocation != null)
    {
      own = mSavedLocation;
      ownErrorM = own.getAccuracy();
    }
    else if (mManualLocation != null)
    {
      // The car has driven on since the mark.
      own = mManualLocation;
      ownErrorM = own.getAccuracy()
                + GpsReturnDetector.MAX_SPEED_MPS * (SystemClock.elapsedRealtime() - mManualSetTimeMs) / 1000.0;
    }
    final int carSpeedKmh = mInertial != null && isInertialNavigationEnabled() ? mInertial.getSpeedKmh() : -1;
    final double carSpeedMps = carSpeedKmh >= 0 ? carSpeedKmh / 3.6 * mInertial.getSpeedScale() : Double.NaN;
    return mGpsReturnDetector.onGpsPosition(gps.getLatitude(), gps.getLongitude(), gps.getAccuracy(),
                                            gps.getElapsedRealtimeNanos() / 1_000_000,
                                            own != null ? own.getLatitude() : 0, own != null ? own.getLongitude() : 0,
                                            ownErrorM, gps.hasSpeed() ? gps.getSpeed() : Double.NaN, carSpeedMps,
                                            isNearRoad(gps));
  }

  /**
   * @return true if the GPS position is good to measure the car speed errors by: GPS is not spoofed, accurate,
   * has worked without breaks and jumps for a while and is on a road. It is used in the manual mode and when
   * GPS is disabled for tests too.
   */
  private boolean isSpeedTableGps(@NonNull Location gps)
  {
    if (sourceOf(gps) != PositionSource.GPS || ageMs(gps) > GPS_FRESH_MS
        || gps.getAccuracy() > SPEED_TABLE_GPS_MAX_ACCURACY_M || mSpoofingDetector.isSpoofed())
    {
      return false;
    }
    return mSpeedTableGpsDetector.onGpsPosition(gps.getLatitude(), gps.getLongitude(), gps.getAccuracy(),
                                                gps.getElapsedRealtimeNanos() / 1_000_000, 0, 0,
                                                Double.NaN /* no own position */, Double.NaN, Double.NaN, false)
        && isNearRoad(gps);
  }

  /**
   * @return how long GPS has not given a trusted position, counted from leaving the manual mode at most.
   */
  private long getGpsSilenceMs()
  {
    return SystemClock.elapsedRealtime() - Math.max(mLastTrustedGpsMs, mManualLeftMs);
  }

  /**
   * @return true if GPS jumps away from the inertial position that has followed it: a car can't do that,
   * GPS is spoofed or broken. It is noticed at once, long before the spoofing detector notices it.
   */
  private boolean contradictsInertial(@NonNull Location gps)
  {
    if (mInertial == null || !mInertial.isReady()
        || SystemClock.elapsedRealtime() - mLastTrustedGpsMs > GPS_FRESH_MS)
      return false;
    final Location inertial = mInertial.getLocation();
    return inertial != null && inertial.distanceTo(gps) > MAX_GPS_INERTIAL_DIFF_M + gps.getAccuracy();
  }

  /**
   * @return true if GPS is away from the roads while the car drives: a car is always on a road, GPS is led
   * away by a spoofer.
   */
  private boolean isOffRoadWhileDriving(@NonNull Location gps)
  {
    if (!RoutingController.get().isNavigating() && (mInertial == null || !mInertial.isReady()))
      return false;
    // A slow car may be in a yard or a parking lot that is not on the map.
    if (!gps.hasSpeed() || gps.getSpeed() < OFF_ROAD_MIN_SPEED_MPS)
      return false;
    return !isNearRoad(gps);
  }

  /**
   * @return true if there is a road within the accuracy of GPS plus a margin, going any way: a car turning at
   * a crossing goes aside from both roads for a moment.
   */
  private static boolean isNearRoad(@NonNull Location gps)
  {
    final double radius = MAX_GPS_OFF_ROAD_M + gps.getAccuracy();
    final double[] road = LocationState.nativeSnapToRoad(gps.getLatitude(), gps.getLongitude(), -1 /* bearing */,
                                                         radius, false /* matchRoute */);
    if (road == null)
      return false;
    final float[] distance = new float[1];
    Location.distanceBetween(gps.getLatitude(), gps.getLongitude(), road[0], road[1], distance);
    return distance[0] <= radius;
  }

  /**
   * GPS is lost or wrong while driving: the manual mode is turned on at once, the car is followed by the
   * inertial navigation and the marks of the user. It is left by itself when GPS is trusted again.
   */
  private void enterManualModeByItself(@NonNull String reason)
  {
    if (mManualMode)
      return;
    Logger.w(TAG, "Entering the manual mode: " + reason);
    final Location last = mSavedLocation;
    setManualMode(true);
    // The car is where it was shown until the user moves it: on the road there, or where GPS has left it if
    // there is no road. The inertial navigation has moved the car on since, it continues from there.
    final boolean inertialReady = mInertial != null && mInertial.isReady();
    if (last != null && !inertialReady && !setManualLocation(last.getLatitude(), last.getLongitude())
        && sourceOf(last) == PositionSource.GPS)
    {
      final Location location = new Location(last);
      location.setProvider(MANUAL_PROVIDER);
      location.setAccuracy(MANUAL_ACCURACY_M);
      mManualLocation = location;
      mManualSetTimeMs = SystemClock.elapsedRealtime();
      applyManualLocation();
    }
    for (ManualModeListener listener : mManualModeListeners)
      listener.onGpsLost();
  }

  /**
   * Turns the manual mode on when there is no GPS while navigating without the inertial navigation, which
   * turns it on by itself. GPS may have been lost or never found since the start.
   */
  private void checkGpsLost()
  {
    if (mManualMode || !RoutingController.get().isNavigating())
      return;
    final long now = SystemClock.elapsedRealtime();
    final boolean lost = mLastTrustedGpsMs == 0 && mManualLeftMs == 0 ? now - mStartTimeMs > GPS_FIRST_FIX_MS
                                                                      : getGpsSilenceMs() > GPS_LOST_MS;
    if (lost)
      enterManualModeByItself("No GPS while navigating");
  }

  @NonNull
  private static PositionSource sourceOf(@NonNull Location location)
  {
    if (MANUAL_PROVIDER.equals(location.getProvider()))
      return PositionSource.MANUAL;
    if (InertialNavigator.PROVIDER.equals(location.getProvider()))
      return PositionSource.INERTIAL;
    if (LocationManager.GPS_PROVIDER.equals(location.getProvider()))
      return PositionSource.GPS;
    if (LocationManager.NETWORK_PROVIDER.equals(location.getProvider())
        || location.getAccuracy() > FUSED_SATELLITE_MAX_ACCURACY_M)
      return PositionSource.NETWORK;
    return PositionSource.GPS;
  }

  private static void rebuildRouteIfOffRoute(@NonNull Location location)
  {
    // The core knows where the car goes from the GPS track, a position set by hand or calculated has no
    // track, so its direction is passed, otherwise the route can turn the car around.
    final PositionSource source = sourceOf(location);
    final boolean hasDirection = location.hasBearing()
                              && (source == PositionSource.MANUAL || source == PositionSource.INERTIAL);
    final float offRouteDistanceM = source == PositionSource.MANUAL
                                      ? MANUAL_OFF_ROUTE_DISTANCE_M
                                      : Math.max(location.getAccuracy(), MIN_OFF_ROUTE_DISTANCE_M);
    LocationState.nativeRebuildRouteIfOffRoute(System.currentTimeMillis(), location.getLatitude(),
                                                location.getLongitude(), offRouteDistanceM,
                                                hasDirection ? location.getBearing() : -1);
  }

  // Used by GoogleFusedLocationProvider.
  @SuppressWarnings("unused")
  @Override
  @UiThread
  public void onLocationResolutionRequired(@NonNull PendingIntent pendingIntent)
  {
    Logger.d(TAG);

    if (!isActive())
    {
      Logger.w(TAG, "Provider is not active");
      return;
    }

    // Stop provider until location resolution is granted.
    stop();
    LocationState.nativeOnLocationError(LocationState.ERROR_GPS_OFF);

    mListenersIterator.rewind();
    while (mListenersIterator.hasNext())
      mListenersIterator.next().onLocationResolutionRequired(pendingIntent);
  }

  // Used by GoogleFusedLocationProvider.
  @SuppressWarnings("unused")
  @RequiresPermission(anyOf = {ACCESS_COARSE_LOCATION, ACCESS_FINE_LOCATION})
  @Override
  @UiThread
  public void onFusedLocationUnsupported()
  {
    // Try to downgrade to the native provider first and restart the service before notifying the user.
    Logger.d(TAG, "provider = " + mLocationProvider.getClass().getSimpleName() + " is not supported,"
                      + " downgrading to use native provider");
    mLocationProvider.stop();
    mLocationProvider = new AndroidNativeProvider(mContext, this);
    mActive = true;
    mLocationProvider.start(mInterval);
  }

  /**
   * @return the last GPS position if GPS works now, also in the manual mode where it is not used, or null.
   */
  @Nullable
  public Location getWorkingGps()
  {
    if (mLastGoodGps == null || mSpoofingDetector.isSpoofed())
      return null;
    return ageMs(mLastGoodGps) <= GPS_STATUS_MAX_AGE_MS ? mLastGoodGps : null;
  }

  /**
   * @return the last cell tower position if it is recent, or null.
   */
  @Nullable
  public Location getWorkingNetwork()
  {
    if (mNetworkLocation == null)
      return null;
    return ageMs(mNetworkLocation) <= NETWORK_STATUS_MAX_AGE_MS ? mNetworkLocation : null;
  }

  private static long ageMs(@NonNull Location location)
  {
    return SystemClock.elapsedRealtime() - location.getElapsedRealtimeNanos() / 1_000_000;
  }

  /**
   * @return where the current position comes from, NONE if there is no position or it is too old.
   */
  @NonNull
  public PositionSource getPositionSource()
  {
    if (isInertialActive())
      return PositionSource.INERTIAL;
    if (mManualMode)
      return mManualLocation != null ? PositionSource.MANUAL : PositionSource.NONE;
    if (mSavedLocation == null || !isActive())
      return PositionSource.NONE;
    final long ageMs = SystemClock.elapsedRealtime() - mSavedLocation.getElapsedRealtimeNanos() / 1_000_000;
    if (ageMs > POSITION_STALE_MS)
      return PositionSource.NONE;
    return sourceOf(mSavedLocation);
  }

  /**
   * @return accuracy of the current position in meters.
   */
  public float getPositionAccuracy()
  {
    return mSavedLocation != null ? mSavedLocation.getAccuracy() : 0;
  }

  /**
   * @return time since the user has set own position in the manual mode.
   */
  public long getManualPositionAgeMs()
  {
    return SystemClock.elapsedRealtime() - mManualSetTimeMs;
  }

  public boolean isGpsSpoofed()
  {
    return mSpoofingDetector.isSpoofed();
  }

  @UiThread
  public void addGpsSpoofingListener(@NonNull GpsSpoofingListener listener)
  {
    mSpoofingListeners.addObserver(listener);
  }

  @UiThread
  public void removeGpsSpoofingListener(@NonNull GpsSpoofingListener listener)
  {
    mSpoofingListeners.removeObserver(listener);
  }

  private void onGpsSpoofingChanged(boolean spoofed)
  {
    Logger.w(TAG, "GPS spoofed = " + spoofed);
    for (GpsSpoofingListener listener : mSpoofingListeners)
      listener.onGpsSpoofingChanged(spoofed);

    if (spoofed && (RoutingController.get().isNavigating() || (mInertial != null && mInertial.isReady())))
    {
      enterManualModeByItself("GPS is spoofed");
      return;
    }

    // Replace the spoofed position with the network one right away instead of waiting for the next update.
    if (spoofed && !mManualMode && mNetworkLocation != null)
    {
      mSavedLocation = mNetworkLocation;
      mMyPosition = null;
      notifyLocationUpdated();
    }
  }

  @SuppressLint("MissingPermission")
  private void startNetworkUpdates()
  {
    final LocationManager locationManager = (LocationManager) mContext.getSystemService(Context.LOCATION_SERVICE);
    if (!locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER))
    {
      Logger.w(TAG, "Network provider is disabled, GPS spoofing detection is limited");
      return;
    }
    // Verify the very first GPS positions too, the fresh network position may arrive only several seconds later.
    final Location lastNetworkLocation = locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
    if (lastNetworkLocation != null && mNetworkLocation == null)
    {
      Logger.i(TAG, "Last known network location = " + lastNetworkLocation);
      mNetworkLocation = lastNetworkLocation;
      mSpoofingDetector.onNetworkPosition(lastNetworkLocation.getLatitude(), lastNetworkLocation.getLongitude(),
                                          lastNetworkLocation.getAccuracy(),
                                          lastNetworkLocation.getElapsedRealtimeNanos() / 1_000_000);
    }
    final LocationRequestCompat request = new LocationRequestCompat.Builder(INTERVAL_NETWORK_MS).build();
    LocationManagerCompat.requestLocationUpdates(locationManager, LocationManager.NETWORK_PROVIDER, request,
                                                 mNetworkListener, Looper.getMainLooper());
  }

  private void stopNetworkUpdates()
  {
    final LocationManager locationManager = (LocationManager) mContext.getSystemService(Context.LOCATION_SERVICE);
    LocationManagerCompat.removeUpdates(locationManager, mNetworkListener);
  }

  public boolean isInertialNavigationEnabled()
  {
    return Config.isInertialNavigationEnabled();
  }

  @UiThread
  public void setInertialNavigationEnabled(boolean enabled)
  {
    Logger.i(TAG, "enabled = " + enabled);
    Config.setInertialNavigationEnabled(enabled);
    if (enabled)
      startInertialNavigation();
    else if (mInertial != null)
      mInertial.stop();
  }

  @UiThread
  public void setElm327Address(@NonNull String address)
  {
    Config.setElm327Address(address);
    if (isInertialNavigationEnabled() && mInertial != null)
      mInertial.start();
  }

  public boolean isEsp32Source()
  {
    return Config.isNoGpsEsp32Source();
  }

  /**
   * Chooses where the car movement comes from: the ESP32 sensor box or the phone with an ELM327 adapter.
   */
  @UiThread
  public void setEsp32Source(boolean esp32)
  {
    Logger.i(TAG, "esp32 = " + esp32);
    Config.setNoGpsEsp32Source(esp32);
    if (isInertialNavigationEnabled() && mInertial != null)
      mInertial.start();
  }

  /**
   * @return the inertial navigation to show its state, null if it has never been enabled.
   */
  @Nullable
  public InertialNavigator getInertialNavigator()
  {
    return mInertial;
  }

  /**
   * Turns the car around on its road: the car looks one of the two ways of the road, the user chooses the
   * other one.
   */
  @UiThread
  public void reverseDirection()
  {
    final Location from = mSavedLocation;
    final double bearing = getCarBearing();
    if (from == null || Double.isNaN(bearing))
      return;

    final float reversed = (float) DeadReckoning.normalize(bearing + 180);
    Logger.i(TAG, "Reversed the car to " + Math.round(reversed));
    if (mInertial != null)
      mInertial.setHeading(reversed, InertialNavigator.HeadingSource.ROAD);
    if (mManualLocation != null)
      mManualLocation.setBearing(reversed);
    // The crossings the position has stopped at are on the other side now.
    final Location stopForward = mShiftStopForward;
    mShiftStopForward = mShiftStopBack;
    mShiftStopBack = stopForward;

    final Location location = new Location(from);
    location.setBearing(reversed);
    location.setTime(System.currentTimeMillis());
    location.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
    mSavedLocation = location;
    mMyPosition = null;
    // The route goes the other way now.
    rebuildRouteIfOffRoute(location);
    notifyLocationUpdated();
  }

  /**
   * @return true if the car movement is paused while the car maneuvers.
   */
  public boolean isPaused()
  {
    return mInertial != null && mInertial.isPaused();
  }

  /**
   * Pauses the movement of the car calculated by the inertial navigation while the car maneuvers, e.g.
   * parks or turns around in several moves: the speed from the car is always positive. It is resumed by
   * itself when the car drives on.
   */
  @UiThread
  public void togglePause()
  {
    if (mInertial != null)
      mInertial.setPaused(!mInertial.isPaused());
  }

  /**
   * @return where the car looks, NaN if it is unknown.
   */
  private double getCarBearing()
  {
    if (mInertial != null && isInertialNavigationEnabled() && !Double.isNaN(mInertial.getHeading()))
      return mInertial.getHeading();
    if (mSavedLocation != null && mSavedLocation.hasBearing())
      return mSavedLocation.getBearing();
    return Double.NaN;
  }

  /**
   * @return true if the position arrow must show the car direction instead of the compass: without GPS the
   * car looks along its road, and the compass of a phone in a car or in hands looks anywhere.
   */
  public boolean isCarHeadingShown()
  {
    if (Double.isNaN(getCarBearing()))
      return false;
    if (mManualMode)
      return true;
    return mInertial != null && isInertialNavigationEnabled()
        && SystemClock.elapsedRealtime() - mLastTrustedGpsMs >= GPS_FRESH_MS;
  }

  private void showCarHeading()
  {
    if (isCarHeadingShown() && Map.isEngineCreated())
      Map.onCompassUpdated(Math.toRadians(getCarBearing()), true /* forceRedraw */);
  }

  /**
   * Writes the whole state of the navigation to the log once a second. Turn the logging on in the
   * settings to keep it in a file and analyse a drive afterwards.
   */
  private void logTrip()
  {
    mHandler.removeCallbacks(mTripLogRunnable);
    if (!isActive())
      return;

    checkGpsLost();

    final StringBuilder line = new StringBuilder("TRIP src=").append(getPositionSource());
    if (mSavedLocation != null)
    {
      line.append(String.format(Locale.US, " pos=%.6f,%.6f acc=%.0f", mSavedLocation.getLatitude(),
                                mSavedLocation.getLongitude(), mSavedLocation.getAccuracy()));
      if (mSavedLocation.hasBearing())
        line.append(" bear=").append(Math.round(mSavedLocation.getBearing()));
      line.append(" age=").append(ageSec(mSavedLocation.getElapsedRealtimeNanos() / 1_000_000));
    }
    line.append(" gpsAge=").append(mLastTrustedGpsMs == 0 ? "-" : ageSec(mLastTrustedGpsMs));
    line.append(" spoofed=").append(mSpoofingDetector.isSpoofed() ? 1 : 0);
    line.append(" paused=").append(isPaused() ? 1 : 0);
    line.append(" nav=").append(RoutingController.get().isNavigating() ? 1 : 0);
    if (mNetworkLocation != null)
    {
      line.append(String.format(Locale.US, " net=%.5f,%.5f acc=%.0f age=%s", mNetworkLocation.getLatitude(),
                                mNetworkLocation.getLongitude(), mNetworkLocation.getAccuracy(),
                                ageSec(mNetworkLocation.getElapsedRealtimeNanos() / 1_000_000)));
    }
    if (mManualLocation != null)
      line.append(" markAge=").append(ageSec(mManualSetTimeMs));
    if (mInertial != null && isInertialNavigationEnabled())
    {
      line.append(" obd=").append(mInertial.getSourceState()).append(" speed=").append(mInertial.getSpeedKmh());
      line.append(String.format(Locale.US, " scale=%.3f", mInertial.getSpeedScale()));
      line.append(" table=").append(mInertial.getSpeedTableRanges());
      line.append(" gyro=").append(mInertial.getCalibrationState());
      line.append(String.format(Locale.US, " turn=%.0f", mInertial.getLastTurnShiftM()));
      final double heading = mInertial.getHeading();
      line.append(" hdg=").append(Double.isNaN(heading) ? "-" : Math.round(heading));
      line.append('/').append(mInertial.getHeadingSource());
      line.append(" road=").append(mInertial.isOnRoad() ? 1 : 0);
      line.append(" lost=").append(mInertial.isRoadLost() ? 1 : 0);
      line.append(String.format(Locale.US, " snap=%.1f", mInertial.getLastSnapShiftM()));
      line.append(String.format(Locale.US, " fix=%.0f ready=%d", mInertial.getDistanceSinceFix(),
                                mInertial.isReady() ? 1 : 0));
    }
    Logger.i(TAG, line.toString());

    mHandler.postDelayed(mTripLogRunnable, TRIP_LOG_INTERVAL_MS);
  }

  @NonNull
  private static String ageSec(long elapsedRealtimeMs)
  {
    return String.format(Locale.US, "%.1f", (SystemClock.elapsedRealtime() - elapsedRealtimeMs) / 1000.0);
  }

  private void startInertialNavigation()
  {
    if (!isInertialNavigationEnabled())
      return;
    if (mInertial == null)
    {
      final InertialNavigator.Listener listener = new InertialNavigator.Listener() {
        @Override
        public void onInertialLocation(@NonNull Location location)
        {
          LocationHelper.this.onInertialLocation(location);
        }

        @Override
        public void onRoadLost()
        {
          for (ManualModeListener l : mManualModeListeners)
            l.onRoadLost();
        }
      };
      mInertial = new InertialNavigator(mContext, listener, roads(true /* matchRoute */));
    }
    mInertial.start();
  }

  /**
   * @return the bearing for the core, where an unknown one is negative: the core is built with fast math,
   * which does not support NaN.
   */
  private static double toNativeBearing(double bearing)
  {
    return Double.isNaN(bearing) ? -1 : bearing;
  }

  /**
   * @param matchRoute prefer the followed route to the roads around, it is where the car really is.
   */
  @NonNull
  private static Roads roads(boolean matchRoute)
  {
    return new Roads() {
      @Nullable
      @Override
      public double[] snap(double lat, double lon, double bearing, double radius)
      {
        return LocationState.nativeSnapToRoad(lat, lon, toNativeBearing(bearing), radius, matchRoute);
      }

      @NonNull
      @Override
      public double[] findCrossings(double lat, double lon, double radius)
      {
        return LocationState.nativeFindRoadCrossings(lat, lon, radius);
      }
    };
  }

  private boolean isInertialActive()
  {
    return mInertial != null && SystemClock.elapsedRealtime() - mLastInertialUsedMs < INERTIAL_ACTIVE_MS;
  }

  private void onInertialLocation(@NonNull Location location)
  {
    // Trusted GPS is better. In the manual mode GPS is ignored, so the inertial navigation is used. The manual
    // mode just left by the user waits for GPS a while, otherwise it would come back before the first GPS position.
    if (!mManualMode
        && (SystemClock.elapsedRealtime() - mLastTrustedGpsMs < GPS_FRESH_MS || getGpsSilenceMs() < GPS_LOST_MS))
    {
      return;
    }
    if (!isActive())
      return;

    // The car is not followed by GPS any more, the user sees it by the manual mode and can mark the car.
    enterManualModeByItself("GPS is lost, the inertial navigation continues");
    mLastInertialUsedMs = SystemClock.elapsedRealtime();
    // The mark follows the car: when the inertial navigation stops, e.g. the adapter is disconnected, the car
    // stays where it has been driven to, not at the old mark behind.
    if (mManualLocation != null)
    {
      mManualLocation.setLatitude(location.getLatitude());
      mManualLocation.setLongitude(location.getLongitude());
      if (location.hasBearing())
        mManualLocation.setBearing(location.getBearing());
    }
    mSavedLocation = location;
    mMyPosition = null;
    if (mLastPositionSource != PositionSource.INERTIAL)
      rebuildRouteIfOffRoute(location);
    mLastPositionSource = PositionSource.INERTIAL;
    notifyLocationUpdated();
  }

  public boolean isGpsDisabled()
  {
    return mGpsDisabled;
  }

  /**
   * Ignores GPS as if it were jammed, e.g. to test the navigation without GPS where GPS works.
   */
  @UiThread
  public void setGpsDisabled(boolean disabled)
  {
    Logger.i(TAG, "disabled = " + disabled);
    mGpsDisabled = disabled;
    mLastGoodGps = null;
  }

  public boolean isManualMode()
  {
    return mManualMode;
  }

  @UiThread
  public void setManualMode(boolean enabled)
  {
    if (mManualMode == enabled)
      return;

    Logger.i(TAG, "enabled = " + enabled);
    mManualMode = enabled;
    if (!enabled)
      mManualLeftMs = SystemClock.elapsedRealtime();
    mManualLocation = null;
    mAcceptNextLocation = !enabled;
    mGpsReturnDetector.reset();
    mOffRoadGpsCount = 0;
    mHandler.removeCallbacks(mManualRepeatRunnable);

    for (ManualModeListener listener : mManualModeListeners)
      listener.onManualModeChanged(enabled);
  }

  @UiThread
  public void addManualModeListener(@NonNull ManualModeListener listener)
  {
    mManualModeListeners.addObserver(listener);
  }

  @UiThread
  public void removeManualModeListener(@NonNull ManualModeListener listener)
  {
    mManualModeListeners.removeObserver(listener);
  }

  /**
   * Sets the user position to the point on the map view in pixels. Works only in the manual mode.
   * @param x, y the point on the map view in pixels.
   * @return false if there is no road at the point: the car is always on a road.
   */
  @UiThread
  public boolean setManualLocationFromScreen(float x, float y)
  {
    final double[] latLon = LocationState.nativeScreenToLatLon(x, y);
    if (!setManualLocation(latLon[0], latLon[1]))
      return false;
    // The map moved away by the user comes back to the car by itself in a while, and the car seems to drive
    // along the road then. It comes back at once instead, the user has just shown where the car is.
    if (LocationState.getMode() == LocationState.NOT_FOLLOW)
      LocationState.nativeSwitchToNextMode();
    return true;
  }

  /**
   * @return false if there is no road at the point: the car is always on a road.
   */
  @UiThread
  public boolean setManualLocation(double lat, double lon)
  {
    if (!mManualMode)
      throw new IllegalStateException("Manual mode is off");

    // The car stands on the axis of a road and looks one of its two ways: the way it has looked before, the
    // user turns it around with the reverse button. The gyroscope has followed the turns since.
    final double heading = getCarBearing();
    // While navigating, the car is on the route and drives the way the route goes.
    double[] road = RoutingController.get().isNavigating()
                        ? LocationState.nativeProjectToRoute(lat, lon, MANUAL_SNAP_RADIUS_M)
                        : null;
    // The car goes against the route: it has turned around, the road is taken the way the car goes.
    if (road != null && !Double.isNaN(heading) && Math.abs(DeadReckoning.angleDiff(road[2], heading)) > 90)
      road = null;
    // The route starts from a position off the roads with a piece across to the road, the car stands
    // along the road and not across it.
    if (road != null && !isAlongRoad(road[0], road[1], road[2]))
      road = null;
    if (road == null)
      road = snapToRoad(lat, lon, heading, MANUAL_SNAP_RADIUS_M, mSavedLocation);
    if (road == null)
    {
      Logger.i(TAG, String.format(Locale.US, "No road at the tap = %.6f,%.6f", lat, lon));
      return false;
    }

    final Location location = new Location(MANUAL_PROVIDER);
    location.setLatitude(road[0]);
    location.setLongitude(road[1]);
    location.setAccuracy(MANUAL_ACCURACY_M);
    location.setBearing((float) road[2]);
    Logger.i(TAG, String.format(Locale.US, "tap = %.6f,%.6f ", lat, lon) + "location = " + location);

    mManualLocation = location;
    mManualSetTimeMs = SystemClock.elapsedRealtime();
    mLastPositionSource = PositionSource.MANUAL;
    rebuildRouteIfOffRoute(location);
    if (mInertial != null)
      mInertial.setRoadPosition(road[0], road[1], road[2]);
    applyManualLocation();
    return true;
  }

  /**
   * @return true if there is a road at the position going the bearing way or the opposite one.
   */
  private static boolean isAlongRoad(double lat, double lon, double bearingDeg)
  {
    final double[] road =
        LocationState.nativeSnapToRoad(lat, lon, toNativeBearing(bearingDeg), ROAD_CHECK_RADIUS_M, false);
    if (road == null)
      return false;
    final float[] distance = new float[1];
    Location.distanceBetween(lat, lon, road[0], road[1], distance);
    final double diff = Math.abs(DeadReckoning.angleDiff(road[2], bearingDeg));
    return distance[0] <= ROAD_CHECK_RADIUS_M && Math.min(diff, 180 - diff) <= MAX_ALONG_ROAD_DIFF_DEG;
  }

  /**
   * Moves a position to the axis of the closest road, a main one rather than a driveway branching off it
   * nearly as close, and looks along it the way closest to the direction of the movement.
   * @param bearingDeg the direction of the movement, NaN if it is unknown. It only tells which way the car
   * looks: for a mark it is the direction before the mark, and preferring a road going that way would take a
   * driveway or a crossing street next to the tapped one.
   * @param turnedFrom where the car was before, if it may have turned to another road since, e.g. a new
   * mark. Null if it goes on along the same road.
   * @return {latitude, longitude, road bearing the car looks at}, or null if there is no road within the
   * radius.
   */
  @Nullable
  private static double[] snapToRoad(double lat, double lon, double bearingDeg, double radiusM,
                                     @Nullable Location turnedFrom)
  {
    final double[] road = LocationState.nativeSnapToMainRoad(lat, lon, radiusM);
    if (road == null)
      return null;

    // Roads are searched in a square, its corners are farther than the radius.
    final float[] shift = new float[1];
    Location.distanceBetween(lat, lon, road[0], road[1], shift);
    if (shift[0] > radiusM)
      return null;

    // Without the direction of the movement the car looks any way, the user turns it around if it is wrong.
    road[2] = RoadWalker.orient(road[2], bearingDeg);
    if (Double.isNaN(bearingDeg) || Math.abs(DeadReckoning.angleDiff(road[2], bearingDeg)) <= MAX_ROAD_BEARING_DIFF_DEG)
      return road;

    // The road is a crossing one, the car has turned to it, a road going the car's way is preferred and would be returned otherwise.
    // It goes away from where it was, to the left or to the right.
    final Location to = new Location(MANUAL_PROVIDER);
    to.setLatitude(road[0]);
    to.setLongitude(road[1]);
    if (turnedFrom.distanceTo(to) >= MIN_TURN_DISTANCE_M
        && Math.abs(DeadReckoning.angleDiff(road[2], turnedFrom.bearingTo(to))) > 90)
    {
      road[2] = DeadReckoning.normalize(road[2] + 180);
    }
    return road;
  }

  /**
   * Moves the position along the route, or along the road if there is no route, e.g. when the calculated
   * position lags behind the car. The position never leaves the road and stops at the closest crossing.
   * @param distanceM meters to move forward, negative to move back.
   * @return the distance the position was moved by, 0 if it can not be moved.
   */
  @UiThread
  public double shiftPosition(double distanceM)
  {
    final boolean forward = distanceM > 0;
    final Location from = mSavedLocation;
    if (from == null || isShiftBlocked(forward))
      return 0;

    // The direction of the movement tells which part of the route the car drives along: the route passes
    // the car several times, e.g. the street it has just turned from is still a part of the route.
    final double movementBearing = getCarBearing();
    double[] shifted =
        LocationState.nativeShiftAlongRoute(from.getLatitude(), from.getLongitude(), toNativeBearing(movementBearing),
                                            distanceM);
    if (shifted == null && !Double.isNaN(movementBearing))
      shifted = RoadWalker.walk(roads(false /* matchRoute */), from.getLatitude(), from.getLongitude(),
                                movementBearing, distanceM);
    if (shifted == null)
      return 0;

    final double applied = shifted[3];
    // The position has stopped at a crossing: the car can go to another street there, so the position is
    // not moved that way any more until the car really leaves the crossing.
    final boolean atCrossing = shifted.length > 4 && shifted[4] != 0;
    // Less than this is no movement at all, and the direction of such a move means nothing.
    if (Math.abs(applied) < MIN_SHIFT_M)
    {
      if (atCrossing)
        blockShift(forward, from);
      return 0;
    }

    final double lat = shifted[0];
    final double lon = shifted[1];
    final double bearing = shifted[2];
    Logger.i(TAG, "Moved the position by " + Math.round(applied) + " m, bearing = " + Math.round(bearing));

    if (mInertial != null)
      mInertial.onPositionCorrected(lat, lon, bearing, applied);

    // The position is set by the user now, so it is as fresh as a manual mark.
    mManualSetTimeMs = SystemClock.elapsedRealtime();
    if (mManualLocation != null)
    {
      mManualLocation.setLatitude(lat);
      mManualLocation.setLongitude(lon);
      mManualLocation.setBearing((float) bearing);
    }

    // The position is set by the user now, whatever it came from before.
    final Location location = new Location(from);
    location.setProvider(MANUAL_PROVIDER);
    location.setLatitude(lat);
    location.setLongitude(lon);
    location.setBearing((float) bearing);
    location.setTime(System.currentTimeMillis());
    location.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
    mSavedLocation = location;
    mLastPositionSource = PositionSource.MANUAL;
    mMyPosition = null;
    if (atCrossing)
      blockShift(forward, location);

    // Moving back returns the car to the passed part of the route, which is cut off, so it is rebuilt.
    rebuildRouteIfOffRoute(location);
    notifyLocationUpdated();
    return applied;
  }

  private void blockShift(boolean forward, @NonNull Location at)
  {
    if (forward)
      mShiftStopForward = new Location(at);
    else
      mShiftStopBack = new Location(at);
  }

  /**
   * @return true if the position has stopped at a turn and must not be moved that way until the car
   * leaves the turn: the car can take another street there.
   * @param forward true for the direction of the movement, false for the opposite one.
   */
  public boolean isShiftBlocked(boolean forward)
  {
    final Location stop = forward ? mShiftStopForward : mShiftStopBack;
    if (stop == null)
      return false;
    if (mSavedLocation == null || stop.distanceTo(mSavedLocation) >= SHIFT_BLOCK_RADIUS_M)
    {
      if (forward)
        mShiftStopForward = null;
      else
        mShiftStopBack = null;
      return false;
    }
    return true;
  }

  private void applyManualLocation()
  {
    mHandler.removeCallbacks(mManualRepeatRunnable);
    if (mManualLocation == null)
      return;

    // The inertial navigation moves the car from the manual mark, it must not be moved back.
    if (!isInertialActive())
    {
      mManualLocation.setTime(System.currentTimeMillis());
      mManualLocation.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
      mSavedLocation = new Location(mManualLocation);
      mMyPosition = null;
      notifyLocationUpdated();
    }

    if (isActive())
      mHandler.postDelayed(mManualRepeatRunnable, MANUAL_REPEAT_INTERVAL_MS);
  }

  // RouteSimulationProvider doesn't really require location permissions.
  @SuppressLint("MissingPermission")
  public void startNavigationSimulation(JunctionInfo[] points)
  {
    Logger.i(TAG);
    mOldLocationProvider = mLocationProvider;
    mLocationProvider.stop();
    mLocationProvider = new RouteSimulationProvider(mContext, this, points);
    mActive = true;
    mLocationProvider.start(mInterval);
  }

  @SuppressLint("MissingPermission")
  public void stopNavigationSimulation()
  {
    Logger.i(TAG);
    mLocationProvider.stop();
    if (mOldLocationProvider == null)
      throw new IllegalStateException("Should be called only after startNavigationSimulation()");
    mLocationProvider = mOldLocationProvider;
    mActive = true;
    mLocationProvider.start(mInterval);
  }

  @Override
  @UiThread
  public void onLocationDisabled()
  {
    Logger.d(TAG, "provider = " + mLocationProvider.getClass().getSimpleName()
                      + " settings = " + LocationUtils.areLocationServicesTurnedOn(mContext));

    stop();
    LocationState.nativeOnLocationError(LocationState.ERROR_GPS_OFF);

    mListenersIterator.rewind();
    while (mListenersIterator.hasNext())
      mListenersIterator.next().onLocationDisabled();
  }

  /**
   * Registers listener to obtain location updates.
   *
   * @param listener    listener to be registered.
   */
  @UiThread
  public void addListener(@NonNull LocationListener listener)
  {
    Logger.d(TAG, "listener: " + listener + " count was: " + mListeners.size());

    mListeners.addObserver(listener);
    if (mSavedLocation != null)
      listener.onLocationUpdated(mSavedLocation);
  }

  /**
   * Removes given location listener.
   * @param listener listener to unregister.
   */
  @UiThread
  public void removeListener(@NonNull LocationListener listener)
  {
    Logger.d(TAG, "listener: " + listener + " count was: " + mListeners.size());
    mListeners.removeObserver(listener);
  }

  private long calcLocationUpdatesInterval()
  {
    if (RoutingController.get().isNavigating())
      return INTERVAL_NAVIGATION_MS;

    if (TrackRecorder.nativeIsTrackRecordingEnabled())
      return INTERVAL_TRACK_RECORDING;

    final int mode = Map.isEngineCreated() ? LocationState.getMode() : LocationState.NOT_FOLLOW_NO_POSITION;
    return switch (mode)
    {
      case LocationState.PENDING_POSITION, LocationState.FOLLOW, LocationState.FOLLOW_AND_ROTATE -> INTERVAL_FOLLOW_MS;
      case LocationState.NOT_FOLLOW, LocationState.NOT_FOLLOW_NO_POSITION -> INTERVAL_NOT_FOLLOW_MS;
      default -> throw new IllegalArgumentException("Unsupported location mode: " + mode);
    };
  }

  /**
   * Restart the location with a new refresh interval if changed.
   */
  @RequiresPermission(anyOf = {ACCESS_COARSE_LOCATION, ACCESS_FINE_LOCATION})
  public void restartWithNewMode()
  {
    if (!isActive())
    {
      start();
      return;
    }

    final long newInterval = calcLocationUpdatesInterval();
    if (newInterval == mInterval)
      return;

    Logger.i(TAG, "update refresh interval: old = " + mInterval + " new = " + newInterval);
    mLocationProvider.stop();
    mInterval = newInterval;
    mLocationProvider.start(newInterval);
  }

  /**
   * Starts polling location updates.
   */
  @RequiresPermission(anyOf = {ACCESS_COARSE_LOCATION, ACCESS_FINE_LOCATION})
  public void start()
  {
    if (isActive())
    {
      Logger.d(TAG, "Already started");
      return;
    }

    Logger.i(TAG);
    checkForAgpsUpdates();

    if (LocationUtils.checkFineLocationPermission(mContext))
      mSensorHelper.start();

    final long oldInterval = mInterval;
    mInterval = calcLocationUpdatesInterval();
    Logger.i(TAG, "provider = " + mLocationProvider.getClass().getSimpleName() + " mInFirstRun = " + mInFirstRun
                      + " oldInterval = " + oldInterval + " interval = " + mInterval);
    mActive = true;
    mStartTimeMs = SystemClock.elapsedRealtime();
    mLocationProvider.start(mInterval);
    mHandler.postDelayed(mLocationTimeoutRunnable, LOCATION_UPDATE_TIMEOUT_MS);
    subscribeToGnssStatusUpdates();
    startNetworkUpdates();
    startInertialNavigation();
    mHandler.post(mTripLogRunnable);
    if (mManualLocation != null)
      mHandler.post(mManualRepeatRunnable);
  }

  /**
   * Stops the polling location updates.
   */
  public void stop()
  {
    if (!isActive())
    {
      Logger.d(TAG, "Already stopped");
      return;
    }

    Logger.i(TAG);
    mLocationProvider.stop();
    unsubscribeFromGnssStatusUpdates();
    stopNetworkUpdates();
    if (mInertial != null)
      mInertial.stop();
    mSensorHelper.stop();
    mHandler.removeCallbacks(mLocationTimeoutRunnable);
    mHandler.removeCallbacks(mManualRepeatRunnable);
    mHandler.removeCallbacks(mTripLogRunnable);
    mActive = false;
  }

  /**
   * Resume location services when entering the foreground.
   */
  public void resumeLocationInForeground()
  {
    if (isActive())
      return;
    else if (!Map.isEngineCreated())
    {
      // LocationState.nativeGetMode() is initialized only after drape creation.
      // https://github.com/organicmaps/organicmaps/issues/1128#issuecomment-1784435190
      Logger.d(TAG, "Engine is not created yet.");
      return;
    }
    else if (LocationState.getMode() == LocationState.NOT_FOLLOW_NO_POSITION)
    {
      Logger.i(TAG, "Location updates are stopped by the user manually.");
      return;
    }
    else if (!LocationUtils.checkLocationPermission(mContext))
    {
      Logger.i(TAG, "Permissions ACCESS_FINE_LOCATION and ACCESS_COARSE_LOCATION are not granted");
      return;
    }

    start();
  }

  private void checkForAgpsUpdates()
  {
    if (!NetworkPolicy.getCurrentNetworkUsageStatus())
      return;

    long previousTimestamp = Config.getAgpsTimestamp();
    long currentTimestamp = System.currentTimeMillis();
    if (previousTimestamp + AGPS_EXPIRATION_TIME_MS > currentTimestamp)
    {
      Logger.d(TAG, "A-GPS should be up to date");
      return;
    }

    Logger.d(TAG, "Requesting new A-GPS data");
    Config.setAgpsTimestamp(currentTimestamp);
    final LocationManager manager = (LocationManager) mContext.getSystemService(Context.LOCATION_SERVICE);
    manager.sendExtraCommand(LocationManager.GPS_PROVIDER, "force_xtra_injection", null);
    manager.sendExtraCommand(LocationManager.GPS_PROVIDER, "force_time_injection", null);
  }

  private void subscribeToGnssStatusUpdates()
  {
    // Subscribe to the low-level GNSS status to keep the green dot location indicator always firing.
    // https://github.com/organicmaps/organicmaps/issues/5999#issuecomment-1793713369
    if (!LocationUtils.checkFineLocationPermission(mContext))
      return;
    final LocationManager locationManager = (LocationManager) mContext.getSystemService(Context.LOCATION_SERVICE);
    LocationManagerCompat.registerGnssStatusCallback(locationManager, ContextCompat.getMainExecutor(mContext),
                                                     mGnssStatusCallback);
  }

  private void unsubscribeFromGnssStatusUpdates()
  {
    final LocationManager locationManager = (LocationManager) mContext.getSystemService(Context.LOCATION_SERVICE);
    LocationManagerCompat.unregisterGnssStatusCallback(locationManager, mGnssStatusCallback);
  }

  @UiThread
  public boolean isInFirstRun()
  {
    return mInFirstRun;
  }

  @UiThread
  public void onEnteredIntoFirstRun()
  {
    Logger.i(TAG);
    mInFirstRun = true;
  }

  @UiThread
  public void onExitFromFirstRun()
  {
    Logger.i(TAG);
    if (!mInFirstRun)
      throw new AssertionError("Must be called only after 'onEnteredIntoFirstRun' method!");

    mInFirstRun = false;

    // If there is a location we need just to pass it to the listeners, so that
    // my position state machine will be switched to the FOLLOW state.
    if (mSavedLocation != null)
    {
      notifyLocationUpdated();
      Logger.d(TAG, "Current location is available, so play the nice zoom animation");
      Framework.nativeRunFirstLaunchAnimation();
    }
  }

  public boolean isGmsLocationProviderAvailable()
  {
    return mLocationProviderFactory.isGmsLocationProviderAvailable(mContext);
  }
}
