package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Moves the car along its road, e.g. when the user corrects the position with the plus and minus buttons
 * without a route. The car never leaves the road: it follows the bends and stops at the first crossing,
 * where it may turn to another street.
 */
public final class RoadWalker
{
  // Short enough to follow the bends of a road.
  static final double STEP_M = 5;
  // The next point of the same road is close, a farther road is another one.
  static final double SNAP_RADIUS_M = 8;
  // A sharper bend within a step is a turn to another road.
  static final double MAX_BEND_DEG = 45;
  // A crossing the car stands at does not stop it, otherwise it could never leave the crossing.
  static final double SAME_CROSSING_M = 1;
  // A crossing this close to the passed piece of the road is on the road.
  static final double CROSSING_ON_ROAD_M = 3;
  // Crossings are searched around the start once, a bit farther than the distance.
  static final double CROSSING_SEARCH_MARGIN_M = 20;

  private RoadWalker() {}

  /**
   * @param bearing the direction the car looks at.
   * @param distanceM meters to move forward, negative to move back.
   * @return {latitude, longitude, bearing the car looks at, applied distance (negative when moved back), 1 if
   * stopped at a crossing and 0 otherwise}, or null if there is no road at the position.
   */
  @Nullable
  public static double[] walk(@NonNull Roads roads, double lat, double lon, double bearing, double distanceM)
  {
    final boolean forward = distanceM >= 0;
    double heading = forward ? bearing : bearing + 180;
    final double[] start = roads.snap(lat, lon, heading, SNAP_RADIUS_M);
    if (start == null)
      return null;

    double posLat = start[0];
    double posLon = start[1];
    heading = orient(start[2], heading);
    final double[] crossings =
        roads.findCrossings(posLat, posLon, Math.abs(distanceM) + CROSSING_SEARCH_MARGIN_M);

    double remaining = Math.abs(distanceM);
    double applied = 0;
    boolean atCrossing = false;
    while (remaining > 0)
    {
      final double step = Math.min(STEP_M, remaining);
      final double[] next = DeadReckoning.move(posLat, posLon, heading, step);
      final double[] road = roads.snap(next[0], next[1], heading, SNAP_RADIUS_M);
      if (road == null)
        break;
      final double roadHeading = orient(road[2], heading);
      if (Math.abs(DeadReckoning.angleDiff(roadHeading, heading)) > MAX_BEND_DEG)
        break;

      final double[] crossing = findCrossingOnPiece(crossings, start, posLat, posLon, road[0], road[1]);
      if (crossing != null)
      {
        applied += distance(posLat, posLon, crossing[0], crossing[1]);
        posLat = crossing[0];
        posLon = crossing[1];
        atCrossing = true;
        break;
      }

      applied += distance(posLat, posLon, road[0], road[1]);
      posLat = road[0];
      posLon = road[1];
      heading = roadHeading;
      remaining -= step;
    }

    final double carBearing = DeadReckoning.normalize(forward ? heading : heading + 180);
    return new double[] {posLat, posLon, carBearing, forward ? applied : -applied, atCrossing ? 1 : 0};
  }

  /**
   * @return the road bearing turned the way closest to the heading: a road goes both ways.
   */
  public static double orient(double roadBearing, double heading)
  {
    if (Double.isNaN(heading) || Math.abs(DeadReckoning.angleDiff(roadBearing, heading)) <= 90)
      return DeadReckoning.normalize(roadBearing);
    return DeadReckoning.normalize(roadBearing + 180);
  }

  @Nullable
  private static double[] findCrossingOnPiece(@NonNull double[] crossings, @NonNull double[] start, double fromLat,
                                              double fromLon, double toLat, double toLon)
  {
    double[] closest = null;
    double closestDistance = Double.MAX_VALUE;
    for (int i = 0; i + 1 < crossings.length; i += 2)
    {
      final double lat = crossings[i];
      final double lon = crossings[i + 1];
      if (distance(lat, lon, start[0], start[1]) < SAME_CROSSING_M)
        continue;
      if (distanceToPiece(lat, lon, fromLat, fromLon, toLat, toLon) > CROSSING_ON_ROAD_M)
        continue;
      final double d = distance(fromLat, fromLon, lat, lon);
      if (d < closestDistance)
      {
        closestDistance = d;
        closest = new double[] {lat, lon};
      }
    }
    return closest;
  }

  static double distance(double lat1, double lon1, double lat2, double lon2)
  {
    final double dLat = Math.toRadians(lat2 - lat1);
    final double dLon = Math.toRadians(lon2 - lon1);
    final double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                   + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.sin(dLon / 2)
                         * Math.sin(dLon / 2);
    return 2 * 6_371_000 * Math.asin(Math.min(1, Math.sqrt(a)));
  }

  /**
   * @return meters from the point to the piece between two points, on a local flat projection.
   */
  static double distanceToPiece(double lat, double lon, double fromLat, double fromLon, double toLat, double toLon)
  {
    final double metersPerDegLat = 111_320;
    final double metersPerDegLon = 111_320 * Math.cos(Math.toRadians(fromLat));
    final double px = (lon - fromLon) * metersPerDegLon;
    final double py = (lat - fromLat) * metersPerDegLat;
    final double dx = (toLon - fromLon) * metersPerDegLon;
    final double dy = (toLat - fromLat) * metersPerDegLat;
    final double lengthSq = dx * dx + dy * dy;
    final double t = lengthSq == 0 ? 0 : Math.max(0, Math.min(1, (px * dx + py * dy) / lengthSq));
    return Math.hypot(px - t * dx, py - t * dy);
  }
}
