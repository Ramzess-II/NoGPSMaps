package app.organicmaps.sdk.location;

import androidx.annotation.IntDef;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.Map;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

public final class LocationState
{
  public static final String LOCATION_TAG = LocationState.class.getSimpleName();

  public interface ModeChangeListener
  {
    // Used by JNI.
    @Keep
    @SuppressWarnings("unused")
    void onMyPositionModeChanged(int newMode);
  }

  @Retention(RetentionPolicy.SOURCE)
  @IntDef({PENDING_POSITION, NOT_FOLLOW_NO_POSITION, NOT_FOLLOW, FOLLOW, FOLLOW_AND_ROTATE})
  @interface Value
  {}

  // These values should correspond to location::EMyPositionMode enum (from platform/location.hpp)
  public static final int PENDING_POSITION = 0;
  public static final int NOT_FOLLOW_NO_POSITION = 1;
  public static final int NOT_FOLLOW = 2;
  public static final int FOLLOW = 3;
  public static final int FOLLOW_AND_ROTATE = 4;

  // These constants should correspond to values defined in platform/location.hpp
  // Leave 0-value as no any error.
  // private static final int ERROR_UNKNOWN = 0;
  // private static final int ERROR_NOT_SUPPORTED = 1;
  public static final int ERROR_DENIED = 2;
  public static final int ERROR_GPS_OFF = 3;
  // public static final int ERROR_TIMEOUT = 4; // Unused on Android (only used on Qt)

  public static native void nativeSwitchToNextMode();
  @Value
  private static native int nativeGetMode();

  public static native void nativeSetListener(@NonNull ModeChangeListener listener);
  public static native void nativeRemoveListener();

  public static native void nativeOnLocationError(int errorCode);

  static native void nativeLocationUpdated(long time, double lat, double lon, float accuracyH, double altitude,
                                           float accuracyV, float speed, float bearing);

  /**
   * Converts a point on the map view (in pixels) to geographic coordinates.
   * @return {latitude, longitude}
   */
  @NonNull
  static native double[] nativeScreenToLatLon(float x, float y);

  /**
   * @param bearing the direction the car goes, negative if it is unknown: the route is built that way.
   */
  static native void nativeRebuildRouteIfOffRoute(long time, double lat, double lon, float accuracy,
                                                  float bearing);

  /**
   * Snaps a position to the closest road, preferring the one going in the bearing direction.
   * @param bearing the direction of the movement, negative if it is unknown.
   * @param matchRoute prefer the followed route to the roads around.
   * @return {latitude, longitude, road bearing}, or null if there is no route or road within the radius.
   */
  @Nullable
  static native double[] nativeSnapToRoad(double lat, double lon, double bearing, double radius, boolean matchRoute);

  /**
   * Moves a position to the closest point of the followed route.
   * @return {latitude, longitude, route bearing}, or null if there is no route within the radius.
   */
  @Nullable
  static native double[] nativeProjectToRoute(double lat, double lon, double radius);

  /**
   * @return {latitude, longitude} pairs of the points where three or more roads meet within the radius.
   */
  @NonNull
  static native double[] nativeFindRoadCrossings(double lat, double lon, double radius);

  /**
   * Moves a position along the followed route, stopping at the closest turn or crossing in both directions.
   * @param bearing where the car looks, negative if it is unknown. Parts of the route going another way are
   * skipped: they are streets the car has already left or has not reached yet.
   * @param distance meters to move forward, negative to move back.
   * @return {latitude, longitude, route bearing, applied distance (negative when moved back), 1 if stopped
   * at a turn or crossing and 0 otherwise}, or null if there is no followed route, the position is not on
   * it or it is moved back from the start of the route.
   */
  @Nullable
  static native double[] nativeShiftAlongRoute(double lat, double lon, double bearing, double distance);

  private LocationState() {}

  @Value
  public static int getMode()
  {
    if (!Map.isEngineCreated())
      throw new IllegalStateException("Location mode is undefined until engine is created");
    return nativeGetMode();
  }

  public static String nameOf(@Value int mode)
  {
    return switch (mode)
    {
      case PENDING_POSITION -> "PENDING_POSITION";
      case NOT_FOLLOW_NO_POSITION -> "NOT_FOLLOW_NO_POSITION";
      case NOT_FOLLOW -> "NOT_FOLLOW";
      case FOLLOW -> "FOLLOW";
      case FOLLOW_AND_ROTATE -> "FOLLOW_AND_ROTATE";
      default -> "Unknown: " + mode;
    };
  }
}
