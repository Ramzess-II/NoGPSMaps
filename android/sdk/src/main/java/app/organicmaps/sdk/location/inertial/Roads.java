package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * The roads of the map, the car is always on one of them.
 */
public interface Roads
{
  /**
   * @param bearing the direction the car looks at, a road going that way is preferred. NaN if it is unknown.
   * @return {latitude, longitude, road bearing} of the closest point of a road, the bearing is for any of the
   * two directions of the road. Null if there is no road within the radius.
   */
  @Nullable
  double[] snap(double lat, double lon, double bearing, double radius);

  /**
   * @return {latitude, longitude} pairs of the points where three or more roads meet within the radius.
   */
  @NonNull
  double[] findCrossings(double lat, double lon, double radius);
}
