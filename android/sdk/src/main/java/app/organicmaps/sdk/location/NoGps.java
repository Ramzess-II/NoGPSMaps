package app.organicmaps.sdk.location;

import android.location.Location;
import android.location.LocationManager;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import androidx.core.location.LocationCompat;

/**
 * Navigation without GPS, done in the core (libs/map/nogps): it chooses the position to show from GPS, cell towers,
 * the marks of the user and the inertial navigation by the car speed and turns. The platform gives it the positions,
 * the sensors and the connections to the car. Everything is called on the UI thread.
 */
public final class NoGps
{
  // The orders of the enums are the same as in the core.

  public enum PositionSource
  {
    NONE,
    GPS,
    // Cell towers and Wi-Fi.
    NETWORK,
    MANUAL,
    // Car speed and turns.
    INERTIAL,
  }

  public enum SourceState
  {
    DISCONNECTED,
    CONNECTING,
    // The ELM327 adapter doesn't answer: it is not plugged in, switched off or broken.
    NO_ADAPTER,
    // The sensor box is set up not to use ELM327.
    OBD_DISABLED,
    // The sensor box is connected, its ELM327 is being initialized or searches the protocol of the car.
    OBD_CONNECTING,
    // ELM327 of the sensor box has a bus error or has stopped answering.
    OBD_ERROR,
    // Connected, but the car doesn't tell its speed (ignition off, protocol search).
    NO_CAR_DATA,
    // The sensor box has gone after the engine was stopped: it switches its Wi-Fi off to save the car battery.
    BOX_SLEEPING,
    // The sensor box is connected and doesn't talk to the car with the engine stopped.
    OBD_SLEEPING,
    CONNECTED,
  }

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

  public enum Event
  {
    MANUAL_MODE_CHANGED,
    // The manual mode has been left by itself: GPS is trusted again.
    GPS_BACK,
    // The manual mode has been turned on by itself: GPS is lost or wrong while driving.
    GPS_LOST,
    GPS_SPOOFED,
    GPS_RESTORED,
    // The inertial navigation has left the roads: the car is stopped on the road, the user should mark it.
    ROAD_LOST,
    // The gyroscope turns the car the other way than GPS: the inertial navigation would turn it wrong.
    TURNS_REVERSED,
    // The car speed has stopped coming while the car is followed without GPS. Repeated while it lasts.
    MOTION_SOURCE_STOPPED,
    // The user has tapped the map in the manual mode where there is no road: the car is always on a road.
    MARK_NO_ROAD,
  }

  // nogps::Provider.
  static final int PROVIDER_GPS = 0;
  static final int PROVIDER_NETWORK = 1;
  static final int PROVIDER_FUSED = 2;
  static final int PROVIDER_MANUAL = 3;
  static final int PROVIDER_INERTIAL = 4;

  public static final String MANUAL_PROVIDER = "manual";
  public static final String INERTIAL_PROVIDER = "inertial";

  /**
   * Everything the UI shows, taken at once. Filled by the core.
   */
  @Keep
  public static final class Status
  {
    @Keep
    int source;
    // Of the shown position.
    @Keep
    public float accuracyM;
    @Keep
    public boolean manualMode;
    // Since the user has set the position in the manual mode.
    @Keep
    public long manualAgeMs;
    @Keep
    public boolean gpsSpoofed;
    @Keep
    public boolean gpsDisabled;
    // GPS works now, also in the manual mode where it is not used. The accuracy is negative if it doesn't.
    @Keep
    public float workingGpsAccuracyM;
    @Keep
    public float workingNetworkAccuracyM;
    // The position is not from GPS: the user corrects it with the buttons.
    @Keep
    public boolean movedByHand;
    // The position stands at a crossing and is moved that way after the car passes it.
    @Keep
    public boolean shiftForwardBlocked;
    @Keep
    public boolean shiftBackBlocked;
    @Keep
    public int shiftStepM;
    @Keep
    public boolean shiftButtonsShown;

    @Keep
    public boolean inertialEnabled;
    @Keep
    public boolean esp32Source;
    // Empty if no adapter is chosen.
    @Keep
    public String elm327Address = "";
    @Keep
    public String esp32Address = "";
    // The rest is known while the inertial navigation works.
    @Keep
    public boolean inertialStarted;
    @Keep
    int sourceState;
    @Keep
    public String deviceName = "";
    // -1 if unknown.
    @Keep
    public int speedKmh = -1;
    @Keep
    public boolean hasCarInfo;
    // -1 if unknown, 0 if stopped, 1 if running.
    @Keep
    public int engineRunning = -1;
    // The car data: -1 if unknown.
    @Keep
    public int rpm = -1;
    @Keep
    public int boxMillivolts = -1;
    @Keep
    public int elmMillivolts = -1;
    @Keep
    public int ecuMillivolts = -1;
    @Keep
    public boolean voltageMismatch;
    @Keep
    public double speedScale = 1;
    @Keep
    public int speedTableRanges;
    @Keep
    public double speedLagSec;
    @Keep
    public boolean speedLagMeasured;
    @Keep
    int calibration;
    @Keep
    public int calibrationProgress;
    @Keep
    public boolean hasInertialPosition;
    @Keep
    public boolean paused;

    @NonNull
    public PositionSource getSource()
    {
      return PositionSource.values()[source];
    }

    @NonNull
    public SourceState getSourceState()
    {
      return SourceState.values()[sourceState];
    }

    @NonNull
    public CalibrationState getCalibration()
    {
      return CalibrationState.values()[calibration];
    }
  }

  /**
   * What the platform does for the core.
   */
  @Keep
  interface Delegate {
    /**
     * The accelerometer, and the gyroscope if asked, to {@link #onAccel} and {@link #onGyro}.
     */
    @Keep
    void startMotionSensors(boolean gyroscope);

    @Keep
    void stopMotionSensors();

    /**
     * The ELM327 adapter over Bluetooth SPP: {@link #onElm327Connected} or {@link #onElm327Closed} follows.
     */
    @Keep
    void elm327Connect(@NonNull String address);

    @Keep
    void elm327Write(@NonNull byte[] data);

    @Keep
    void elm327Close();

    /**
     * The ESP32 sensor box over UDP on its Wi-Fi network: the datagrams go to {@link #onEsp32Datagram}.
     */
    @Keep
    void esp32Open(@NonNull String host, int port);

    @Keep
    void esp32Send(@NonNull byte[] line);

    @Keep
    void esp32Close();

    /**
     * The position to show and to navigate by.
     */
    @Keep
    void onPosition(@NonNull Location location);

    @Keep
    void onEvent(int event);
  }

  private NoGps() {}

  @UiThread
  static void create(@NonNull Delegate delegate)
  {
    nativeCreate(delegate);
  }

  @UiThread
  static void onLocation(@NonNull Location location, boolean lastKnownNetwork)
  {
    final int provider;
    if (LocationManager.GPS_PROVIDER.equals(location.getProvider()))
      provider = PROVIDER_GPS;
    else if (LocationManager.NETWORK_PROVIDER.equals(location.getProvider()))
      provider = PROVIDER_NETWORK;
    else
      provider = PROVIDER_FUSED;
    final LocationCompatExtractor.Altitude altitude = LocationCompatExtractor.getAltitude(location);
    nativeOnFix(provider, location.getLatitude(), location.getLongitude(), location.getAccuracy(),
                location.hasBearing(), location.getBearing(), LocationCompat.hasBearingAccuracy(location),
                LocationCompat.hasBearingAccuracy(location) ? LocationCompat.getBearingAccuracyDegrees(location) : 0,
                location.hasSpeed(), location.getSpeed(), altitude != null, altitude != null ? altitude.altitude() : 0,
                altitude != null ? altitude.accuracy() : -1, location.getElapsedRealtimeNanos() / 1_000_000,
                location.getTime(), lastKnownNetwork);
  }

  /**
   * @return the location the core has sent to the platform.
   */
  @NonNull
  @Keep
  @SuppressWarnings("unused") // Called from JNI.
  static Location createLocation(int provider, double lat, double lon, float accuracy, boolean hasBearing,
                                 float bearing, boolean hasSpeed, float speed, boolean hasAltitude, double altitude,
                                 float altitudeAccuracy, long elapsedMs, long unixMs)
  {
    final String name = switch (provider)
    {
      case PROVIDER_GPS -> LocationManager.GPS_PROVIDER;
      case PROVIDER_NETWORK -> LocationManager.NETWORK_PROVIDER;
      case PROVIDER_MANUAL -> MANUAL_PROVIDER;
      case PROVIDER_INERTIAL -> INERTIAL_PROVIDER;
      default -> LocationUtils.FUSED_PROVIDER;
    };
    final Location location = new Location(name);
    location.setLatitude(lat);
    location.setLongitude(lon);
    location.setAccuracy(accuracy);
    if (hasBearing)
      location.setBearing(bearing);
    if (hasSpeed)
      location.setSpeed(speed);
    if (hasAltitude)
      location.setAltitude(altitude);
    if (altitudeAccuracy >= 0)
      LocationCompat.setVerticalAccuracyMeters(location, altitudeAccuracy);
    location.setTime(unixMs);
    location.setElapsedRealtimeNanos(elapsedMs * 1_000_000);
    return location;
  }

  @NonNull
  public static Status getStatus()
  {
    final Status status = new Status();
    nativeGetStatus(status);
    return status;
  }

  @Nullable
  static Event toEvent(int event)
  {
    final Event[] events = Event.values();
    return event >= 0 && event < events.length ? events[event] : null;
  }

  private static native void nativeCreate(@NonNull Delegate delegate);

  static native void nativeStart();

  static native void nativeStop();

  private static native void nativeOnFix(int provider, double lat, double lon, float accuracy, boolean hasBearing,
                                         float bearing, boolean hasBearingAccuracy, float bearingAccuracy,
                                         boolean hasSpeed, float speed, boolean hasAltitude, double altitude,
                                         float altitudeAccuracy, long elapsedMs, long unixMs, boolean lastKnownNetwork);

  static native void nativeOnGyro(long timestampNs, float x, float y, float z);

  static native void nativeOnAccel(long timestampNs, float x, float y, float z);

  static native void nativeOnElm327Connected();

  static native void nativeOnElm327Bytes(@NonNull byte[] data);

  static native void nativeOnElm327Closed(@NonNull String reason);

  static native void nativeOnEsp32Datagram(@NonNull byte[] data);

  public static native boolean nativeIsManualMode();

  public static native void nativeSetManualMode(boolean enabled);

  public static native double nativeShiftPosition(double distanceM);

  public static native void nativeReverseDirection();

  public static native void nativeTogglePause();

  public static native void nativeSetGpsDisabled(boolean disabled);

  public static native void nativeSetInertialNavigationEnabled(boolean enabled);

  public static native void nativeSetElm327Address(@NonNull String address);

  public static native void nativeSetEsp32Source(boolean esp32);

  public static native void nativeSetEsp32Address(@NonNull String address);

  public static native void nativeCalibrate();

  public static native void nativeClearSpeedCalibration();

  public static native void nativeCycleShiftStep();

  public static native void nativeSetShiftButtonsShown(boolean shown);

  private static native void nativeGetStatus(@NonNull Status status);
}
