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
import app.organicmaps.sdk.routing.JunctionInfo;
import app.organicmaps.sdk.routing.RoutingController;
import app.organicmaps.sdk.util.Config;
import app.organicmaps.sdk.util.NetworkPolicy;
import app.organicmaps.sdk.util.log.Logger;
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

  // Network (cell towers and Wi-Fi) positions are used to detect spoofed GPS.
  private static final long INTERVAL_NETWORK_MS = 5000;

  // A position without updates for longer than this is shown as lost.
  private static final long POSITION_STALE_MS = 30 * 1000;
  // Less accurate positions come from cell towers and Wi-Fi even if reported by the fused provider.
  private static final float SATELLITE_MAX_ACCURACY_M = 100;
  // A position closer to the route than this is considered on the route even if it is very accurate.
  private static final float MIN_OFF_ROUTE_DISTANCE_M = 50;

  public enum PositionSource
  {
    NONE,
    GPS,
    // Cell towers and Wi-Fi.
    NETWORK,
    MANUAL
    // TODO: INERTIAL (car speed from OBD + gyroscope).
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
  // Elapsed realtime of the last position set by the user.
  private long mManualSetTimeMs;
  // After the manual mode the next real position must replace the manual one, even if it is less accurate.
  private boolean mAcceptNextLocation;
  // The source of the last position passed to the core, used to check the route when the source changes.
  @NonNull
  private PositionSource mLastPositionSource = PositionSource.NONE;
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

    if (mManualMode)
    {
      Logger.d(TAG, "Manual mode is on, ignoring location = " + location);
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
    if (LocationManager.NETWORK_PROVIDER.equals(location.getProvider())
        || location.getAccuracy() > SATELLITE_MAX_ACCURACY_M)
      return PositionSource.NETWORK;
    return PositionSource.GPS;
  }

  private static void rebuildRouteIfOffRoute(@NonNull Location location)
  {
    LocationState.nativeRebuildRouteIfOffRoute(System.currentTimeMillis(), location.getLatitude(),
                                                location.getLongitude(),
                                                Math.max(location.getAccuracy(), MIN_OFF_ROUTE_DISTANCE_M));
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

    final Location location = new Location(MANUAL_PROVIDER);
    location.setLatitude(lat);
    location.setLongitude(lon);
    location.setAccuracy(MANUAL_ACCURACY_M);

    // The direction of movement is unknown, estimate it from the previous manual position.
    if (mManualLocation != null)
    {
      if (mManualLocation.distanceTo(location) >= MANUAL_MIN_BEARING_DISTANCE_M)
        location.setBearing(mManualLocation.bearingTo(location));
      else if (mManualLocation.hasBearing())
        location.setBearing(mManualLocation.getBearing());
    }

    Logger.i(TAG, "location = " + location);
    mManualLocation = location;
    mManualSetTimeMs = SystemClock.elapsedRealtime();
    mLastPositionSource = PositionSource.MANUAL;
    rebuildRouteIfOffRoute(location);
    applyManualLocation();
  }

  private void applyManualLocation()
  {
    mHandler.removeCallbacks(mManualRepeatRunnable);
    if (mManualLocation == null)
      return;

    mManualLocation.setTime(System.currentTimeMillis());
    mManualLocation.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
    mSavedLocation = new Location(mManualLocation);
    mMyPosition = null;
    notifyLocationUpdated();

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
    mSensorHelper.stop();
    mHandler.removeCallbacks(mLocationTimeoutRunnable);
    mHandler.removeCallbacks(mManualRepeatRunnable);
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
