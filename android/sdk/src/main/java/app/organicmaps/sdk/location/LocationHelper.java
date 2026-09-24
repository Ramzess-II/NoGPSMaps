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
  // Taps closer than this are too noisy to calculate a movement direction.
  private static final float MANUAL_MIN_BEARING_DISTANCE_M = 30;
  // Older marks tell nothing about the current direction of movement.
  private static final long MANUAL_MAX_BEARING_AGE_MS = 3 * 60 * 1000;
  // The car stands on a road, not between houses, but a farther road is not the road the user means.
  private static final double MANUAL_SNAP_RADIUS_M = 30;
  // A road going another way is a crossing road: the car is on it, but it does not drive along it.
  private static final double MAX_ROAD_BEARING_DIFF_DEG = 45;
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
  // Trusted GPS positions are preferred over the inertial ones while they are this fresh.
  private static final long GPS_FRESH_MS = 3000;
  // The inertial navigation is considered active while its positions are used this recently.
  private static final long INERTIAL_ACTIVE_MS = 1000;
  // Old positions (e.g. the last known one after a provider restart) tell nothing about the car now.
  private static final long GPS_MAX_AGE_MS = 5000;
  // The state is written to the log every second, so a drive can be analysed afterwards.
  private static final long TRIP_LOG_INTERVAL_MS = 1000;

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
  @Nullable
  private Location mManualLocation;
  private final Runnable mManualRepeatRunnable = this::applyManualLocation;
  private final Runnable mTripLogRunnable = this::logTrip;
  // Elapsed realtime of the last position set by the user.
  private long mManualSetTimeMs;
  // The last mark tapped by the user and its elapsed realtime: unlike the manual position, it is not moved
  // by the plus and minus buttons, two marks one after another tell where the car goes.
  @Nullable
  private Location mLastMark;
  private long mLastMarkTimeMs;
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
  private long mLastInertialUsedMs;
  private final ObserverList<ManualModeListener> mManualModeListeners = new ObserverList<>();

  private final GpsSpoofingDetector mSpoofingDetector = new GpsSpoofingDetector();
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
    showInertialHeading();
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

    final boolean wasSpoofed = mSpoofingDetector.isSpoofed();
    final boolean isNetwork = LocationManager.NETWORK_PROVIDER.equals(location.getProvider());
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

    if (!trusted)
    {
      Logger.w(TAG, "Untrusted location = " + location);
      return;
    }

    final long ageMs = SystemClock.elapsedRealtime() - location.getElapsedRealtimeNanos() / 1_000_000;
    if (!mManualMode && sourceOf(location) == PositionSource.GPS && ageMs <= GPS_MAX_AGE_MS
        && LocationUtils.isAccuracySatisfied(location))
    {
      mLastTrustedGpsMs = SystemClock.elapsedRealtime();
      if (mInertial != null)
        mInertial.onReferencePosition(location, true /* isGps */);
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
    LocationState.nativeRebuildRouteIfOffRoute(System.currentTimeMillis(), location.getLatitude(),
                                                location.getLongitude(),
                                                Math.max(location.getAccuracy(), MIN_OFF_ROUTE_DISTANCE_M),
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
      mInertial.start(address);
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
   * Sets the direction the car looks at by a point on the map view in pixels.
   */
  @UiThread
  public void setHeadingFromScreen(float x, float y)
  {
    if (mInertial == null)
      return;
    // The direction is set from the arrow the user sees.
    final Location from = mSavedLocation;
    if (from == null)
      return;
    final double[] latLon = LocationState.nativeScreenToLatLon(x, y);
    final Location to = new Location(MANUAL_PROVIDER);
    to.setLatitude(latLon[0]);
    to.setLongitude(latLon[1]);
    mInertial.setPosition(from.getLatitude(), from.getLongitude());
    mInertial.setHeading(from.bearingTo(to), InertialNavigator.HeadingSource.USER);
    showInertialHeading();
    applyInertialHeadingToPosition();
  }

  /**
   * Turns the car direction for the inertial navigation.
   * @param deltaDeg clockwise rotation.
   */
  @UiThread
  public void rotateHeading(double deltaDeg)
  {
    if (mInertial == null)
      return;
    mInertial.rotateHeading(deltaDeg);
    showInertialHeading();
    applyInertialHeadingToPosition();
  }

  /**
   * @return true if the position arrow must show the inertial heading instead of the compass: the compass is
   * unreliable in a car, and the user sets the car direction by the arrow.
   */
  public boolean isInertialHeadingShown()
  {
    if (mInertial == null || !isInertialNavigationEnabled() || Double.isNaN(mInertial.getHeading()))
      return false;
    return mManualMode || SystemClock.elapsedRealtime() - mLastTrustedGpsMs >= GPS_FRESH_MS;
  }

  private void showInertialHeading()
  {
    if (isInertialHeadingShown() && Map.isEngineCreated())
      Map.onCompassUpdated(Math.toRadians(mInertial.getHeading()), true /* forceRedraw */);
  }

  /**
   * Sends the car direction to the map together with the position: in the navigation mode the map takes the
   * direction of the arrow from the position only, the compass is ignored there.
   */
  private void applyInertialHeadingToPosition()
  {
    if (mSavedLocation == null || !isInertialHeadingShown())
      return;

    final float heading = (float) mInertial.getHeading();
    if (mManualLocation != null)
      mManualLocation.setBearing(heading);
    final Location location = new Location(mSavedLocation);
    location.setBearing(heading);
    location.setTime(System.currentTimeMillis());
    location.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
    mSavedLocation = location;
    mMyPosition = null;
    notifyLocationUpdated();
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
      line.append(" obd=").append(mInertial.getElm327State()).append(" speed=").append(mInertial.getSpeedKmh());
      line.append(String.format(Locale.US, " scale=%.3f", mInertial.getSpeedScale()));
      line.append(" gyro=").append(mInertial.getCalibrationState());
      final double heading = mInertial.getHeading();
      line.append(" hdg=").append(Double.isNaN(heading) ? "-" : Math.round(heading));
      line.append('/').append(mInertial.getHeadingSource());
      line.append(" road=").append(mInertial.isOnRoad() ? 1 : 0);
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
      mInertial = new InertialNavigator(
          mContext, this::onInertialLocation,
          (lat, lon, bearing, radius)
              -> LocationState.nativeSnapToRoad(lat, lon, bearing, radius, true /* matchRoute */));
    mInertial.start(Config.getElm327Address());
  }

  private boolean isInertialActive()
  {
    return mInertial != null && SystemClock.elapsedRealtime() - mLastInertialUsedMs < INERTIAL_ACTIVE_MS;
  }

  private void onInertialLocation(@NonNull Location location)
  {
    // Trusted GPS is better. In the manual mode GPS is ignored, so the inertial navigation is used.
    if (!mManualMode && SystemClock.elapsedRealtime() - mLastTrustedGpsMs < GPS_FRESH_MS)
      return;
    if (!isActive())
      return;

    mLastInertialUsedMs = SystemClock.elapsedRealtime();
    mSavedLocation = location;
    mMyPosition = null;
    if (mLastPositionSource != PositionSource.INERTIAL)
      rebuildRouteIfOffRoute(location);
    mLastPositionSource = PositionSource.INERTIAL;
    notifyLocationUpdated();
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
    mManualLocation = null;
    mLastMark = null;
    mAcceptNextLocation = !enabled;
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
   * Sets the user position to the point on the map view. Works only in the manual mode.
   * @param x, y the point on the map view in pixels.
   */
  @UiThread
  public void setManualLocationFromScreen(float x, float y)
  {
    final double[] latLon = LocationState.nativeScreenToLatLon(x, y);
    setManualLocation(latLon[0], latLon[1]);
  }

  @UiThread
  public void setManualLocation(double lat, double lon)
  {
    if (!mManualMode)
      throw new IllegalStateException("Manual mode is off");

    // The car stands along a road, not between houses: the tap is moved to the axis of the closest road and
    // the direction of the movement is taken from that road, the way the car goes.
    // Two marks one after another tell that way, a mark behind the previous one means the car has turned
    // around. It is known better than the calculated direction, which does not see a turn around without
    // the gyroscope.
    final double marksBearing = bearingFromLastMark(lat, lon);
    double bearing = Double.isNaN(marksBearing) ? guessMovementBearing(lat, lon) : marksBearing;
    // While navigating, the car is on the route and drives the way the route goes: the guessed direction
    // can be wrong after a turn or an old mark.
    double[] road = RoutingController.get().isNavigating()
                        ? LocationState.nativeProjectToRoute(lat, lon, MANUAL_SNAP_RADIUS_M)
                        : null;
    // The car goes against the route: it has turned around, the road is taken the way the marks go.
    if (road != null && !Double.isNaN(marksBearing) && Math.abs(DeadReckoning.angleDiff(road[2], marksBearing)) > 90)
      road = null;
    if (road == null)
      road = snapToRoad(lat, lon, bearing, MANUAL_SNAP_RADIUS_M);
    if (road != null)
    {
      lat = road[0];
      lon = road[1];
      bearing = road[2];
    }

    final Location location = new Location(MANUAL_PROVIDER);
    location.setLatitude(lat);
    location.setLongitude(lon);
    location.setAccuracy(MANUAL_ACCURACY_M);
    if (!Double.isNaN(bearing))
      location.setBearing((float) bearing);

    Logger.i(TAG, "location = " + location + " on a road = " + (road != null) + " marks bearing = " + marksBearing);
    mManualLocation = location;
    mLastMark = new Location(location);
    mLastMarkTimeMs = SystemClock.elapsedRealtime();
    mManualSetTimeMs = SystemClock.elapsedRealtime();
    mLastPositionSource = PositionSource.MANUAL;
    rebuildRouteIfOffRoute(location);
    if (mInertial != null)
    {
      if (road != null && !Double.isNaN(bearing))
        mInertial.setRoadPosition(lat, lon, bearing);
      else
        mInertial.onReferencePosition(location, false /* isGps */);
    }
    applyManualLocation();
  }

  /**
   * @return the direction the car moves in, NaN if it is unknown.
   */
  /**
   * @return the direction from the previous mark to a new one, NaN if there is no recent mark or it is so
   * close that the new mark is a correction of the same place.
   */
  private double bearingFromLastMark(double lat, double lon)
  {
    if (mLastMark == null || SystemClock.elapsedRealtime() - mLastMarkTimeMs >= MANUAL_MAX_BEARING_AGE_MS)
      return Double.NaN;
    final Location to = new Location(MANUAL_PROVIDER);
    to.setLatitude(lat);
    to.setLongitude(lon);
    if (mLastMark.distanceTo(to) < MANUAL_MIN_BEARING_DISTANCE_M)
      return Double.NaN;
    return DeadReckoning.normalize(mLastMark.bearingTo(to));
  }

  private double guessMovementBearing(double lat, double lon)
  {
    if (mInertial != null && isInertialNavigationEnabled() && !Double.isNaN(mInertial.getHeading())
        && mInertial.getHeadingSource() != InertialNavigator.HeadingSource.MANUAL_MARKS)
      return mInertial.getHeading();

    if (mManualLocation != null && SystemClock.elapsedRealtime() - mManualSetTimeMs < MANUAL_MAX_BEARING_AGE_MS)
    {
      final Location to = new Location(MANUAL_PROVIDER);
      to.setLatitude(lat);
      to.setLongitude(lon);
      if (mManualLocation.distanceTo(to) >= MANUAL_MIN_BEARING_DISTANCE_M)
        return DeadReckoning.normalize(mManualLocation.bearingTo(to));
      if (mManualLocation.hasBearing())
        return mManualLocation.getBearing();
    }

    if (mSavedLocation != null && mSavedLocation.hasBearing())
      return mSavedLocation.getBearing();

    return Double.NaN;
  }

  /**
   * Moves a position to the axis of the closest road, preferring the one going in the direction of the
   * movement.
   * @return {latitude, longitude, road bearing}, the bearing is NaN if the direction is unknown, or null
   * if there is no road within the radius.
   */
  @Nullable
  private static double[] snapToRoad(double lat, double lon, double bearingDeg, double radiusM)
  {
    final double[] road = LocationState.nativeSnapToRoad(lat, lon, bearingDeg, radiusM, false /* matchRoute */);
    if (road == null)
      return null;

    // Roads are searched in a square, its corners are farther than the radius.
    final float[] shift = new float[1];
    Location.distanceBetween(lat, lon, road[0], road[1], shift);
    if (shift[0] > radiusM)
      return null;

    // Without the direction of the movement it is unknown which way along the road the car looks.
    if (Double.isNaN(bearingDeg))
    {
      road[2] = Double.NaN;
      return road;
    }

    // The road goes in the direction of the movement, but it can be returned in the opposite one.
    if (Math.abs(DeadReckoning.angleDiff(road[2], bearingDeg)) > 90)
      road[2] = DeadReckoning.normalize(road[2] + 180);
    // The closest road can be a crossing one, then it only tells where the car is, not where it looks.
    if (Math.abs(DeadReckoning.angleDiff(road[2], bearingDeg)) > MAX_ROAD_BEARING_DIFF_DEG)
      road[2] = bearingDeg;
    return road;
  }

  /**
   * Moves the position along the route, or along the direction of the movement if there is no route, e.g.
   * when the calculated position lags behind the car. The movement stops at the closest turn.
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
    final double movementBearing = guessMovementBearing(from.getLatitude(), from.getLongitude());
    double[] shifted =
        LocationState.nativeShiftAlongRoute(from.getLatitude(), from.getLongitude(), movementBearing, distanceM);
    if (shifted == null)
      shifted = shiftWithoutRoute(from, movementBearing, distanceM);
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

  /**
   * Moves a position along the direction of the movement, snapping it to a road.
   * @return {latitude, longitude, bearing, applied distance}, or null if the direction is unknown.
   */
  @Nullable
  private double[] shiftWithoutRoute(@NonNull Location from, double bearing, double distanceM)
  {
    if (Double.isNaN(bearing))
      return null;

    final double[] moved = DeadReckoning.move(from.getLatitude(), from.getLongitude(),
                                              distanceM >= 0 ? bearing : bearing + 180, Math.abs(distanceM));
    final double[] road = snapToRoad(moved[0], moved[1], bearing, MANUAL_SNAP_RADIUS_M);
    if (road == null)
      return new double[] {moved[0], moved[1], bearing, distanceM};
    return new double[] {road[0], road[1], road[2], distanceM};
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
