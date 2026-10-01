package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Finds the crossing the car has turned at. The gyroscope tells the turn exactly, but the calculated
 * distance lags behind or runs ahead of the car, so the calculated turn is before or after the real crossing
 * and the car would leave the road after it. The real crossing is the one on the road the car has driven
 * along with a road going the new way.
 */
public final class TurnMatcher
{
  // Smaller turns are bends of the road.
  static final double MIN_TURN_DEG = 35;
  // Larger turns are turns around, not at a crossing.
  static final double MAX_TURN_DEG = 150;
  // The crossing is searched this far along the road at least, the calculated distance is often wrong by
  // tens of meters.
  static final double MIN_SEARCH_M = 60;
  static final double MAX_SEARCH_M = 300;
  // A crossing farther from the road the car drove along is on a parallel street.
  static final double MAX_SIDE_OFFSET_M = 20;
  // The roads of a crossing are checked this far from it: close to it all its roads meet.
  static final double ROAD_CHECK_DISTANCE_M = 12;
  static final double ROAD_CHECK_RADIUS_M = 6;
  static final double MAX_ROAD_DIFF_DEG = 35;
  // The road the car was on going the new way within this difference has bent, the car has followed it.
  static final double MAX_BEND_END_DIFF_DEG = 25;

  private TurnMatcher() {}

  /**
   * @param cornerLat, cornerLon where the car has turned by the calculation.
   * @param fromBearing where the car looked before the turn.
   * @param toBearing where the car looks after the turn.
   * @param searchM how far along the road the crossing is searched, e.g. the accuracy of the calculation.
   * @return {latitude, longitude} of the crossing, or null if the turn is not at a crossing.
   */
  @Nullable
  public static double[] findCrossing(@NonNull Roads roads, double cornerLat, double cornerLon, double fromBearing,
                                      double toBearing, double searchM)
  {
    final double turn = Math.abs(DeadReckoning.angleDiff(toBearing, fromBearing));
    if (turn < MIN_TURN_DEG || turn > MAX_TURN_DEG)
      return null;

    final double radius = Math.max(MIN_SEARCH_M, Math.min(MAX_SEARCH_M, searchM));
    // A bend of the road with a side street close to it would move the car to the street.
    if (isBend(roads, cornerLat, cornerLon, fromBearing, toBearing, radius))
      return null;
    final double[] crossings = roads.findCrossings(cornerLat, cornerLon, radius);
    double[] best = null;
    double bestAlong = Double.MAX_VALUE;
    for (int i = 0; i + 1 < crossings.length; i += 2)
    {
      final double lat = crossings[i];
      final double lon = crossings[i + 1];
      // The offset of the crossing along the road the car came by and to the side of it.
      final double distance = RoadWalker.distance(cornerLat, cornerLon, lat, lon);
      final double bearing = bearing(cornerLat, cornerLon, lat, lon);
      final double angle = Math.toRadians(DeadReckoning.angleDiff(bearing, fromBearing));
      final double along = Math.abs(distance * Math.cos(angle));
      final double side = Math.abs(distance * Math.sin(angle));
      if (along > radius || side > MAX_SIDE_OFFSET_M || along >= bestAlong)
        continue;
      if (!hasRoad(roads, lat, lon, fromBearing + 180) || !hasRoad(roads, lat, lon, toBearing))
        continue;
      best = new double[] {lat, lon};
      bestAlong = along;
    }
    return best;
  }

  /**
   * Follows the road the car was on from the corner: the road going on the way the car looked before. At a
   * crossing it goes straight on or ends.
   * @return true if the road bends to the new direction within the distance.
   */
  static boolean isBend(@NonNull Roads roads, double lat, double lon, double fromBearing, double toBearing,
                        double distanceM)
  {
    final double[] start = roads.snap(lat, lon, fromBearing, RoadWalker.SNAP_RADIUS_M);
    if (start == null)
      return false;
    double posLat = start[0];
    double posLon = start[1];
    double heading = RoadWalker.orient(start[2], fromBearing);
    for (double walked = 0; walked < distanceM; walked += RoadWalker.STEP_M)
    {
      if (Math.abs(DeadReckoning.angleDiff(toBearing, heading)) <= MAX_BEND_END_DIFF_DEG)
        return true;
      final double[] next = DeadReckoning.move(posLat, posLon, heading, RoadWalker.STEP_M);
      final double[] road = roads.snap(next[0], next[1], heading, RoadWalker.SNAP_RADIUS_M);
      if (road == null)
        return false;
      final double roadHeading = RoadWalker.orient(road[2], heading);
      if (Math.abs(DeadReckoning.angleDiff(roadHeading, heading)) > RoadWalker.MAX_BEND_DEG)
        return false;
      posLat = road[0];
      posLon = road[1];
      heading = roadHeading;
    }
    return Math.abs(DeadReckoning.angleDiff(toBearing, heading)) <= MAX_BEND_END_DIFF_DEG;
  }

  /**
   * @return true if a road leaves the crossing in the direction.
   */
  private static boolean hasRoad(@NonNull Roads roads, double lat, double lon, double bearing)
  {
    final double[] point = DeadReckoning.move(lat, lon, bearing, ROAD_CHECK_DISTANCE_M);
    final double[] road = roads.snap(point[0], point[1], bearing, ROAD_CHECK_RADIUS_M);
    if (road == null || RoadWalker.distance(point[0], point[1], road[0], road[1]) > ROAD_CHECK_RADIUS_M)
      return false;
    return Math.abs(DeadReckoning.angleDiff(RoadWalker.orient(road[2], bearing), bearing)) <= MAX_ROAD_DIFF_DEG;
  }

  static double bearing(double fromLat, double fromLon, double toLat, double toLon)
  {
    final double lat1 = Math.toRadians(fromLat);
    final double lat2 = Math.toRadians(toLat);
    final double dLon = Math.toRadians(toLon - fromLon);
    final double y = Math.sin(dLon) * Math.cos(lat2);
    final double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    return DeadReckoning.normalize(Math.toDegrees(Math.atan2(y, x)));
  }
}
