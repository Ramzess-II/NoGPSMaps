package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;
import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * Finds how far the calculated position is ahead of the car or behind it along the road: the car turns where
 * the road bends, so the rotation measured by the gyroscope is compared with the shape of the road around the
 * position. Bends are much more frequent than turns at crossings, where the distance error is removed too.
 * Changing a lane rotates the car by a few degrees and back, it doesn't look like a bend.
 */
public final class BendMatcher
{
  // The rotation of the car is compared with the road along this distance.
  static final double WINDOW_M = 150;
  static final double STEP_M = 5;
  // A straighter road tells nothing about the position along it.
  static final double MIN_BEND_DEG = 20;
  // The shape of the road on the map is rough, a larger difference is another road, e.g. after a fork.
  static final double MAX_DIFF_DEG = 4;
  // The match is clear if the positions this far aside of the best one fit the road much worse. On a
  // roundabout or a long even bend they fit the same.
  static final double CLEAR_ASIDE_M = 15;
  static final double CLEAR_RATIO = 2;
  static final double MIN_ASIDE_DIFF_DEG = 1;
  private static final double SHIFT_STEP_M = 1;
  private static final double HISTORY_STEP_M = 2;
  private static final double MAX_ROAD_BEND_DEG = 45;
  private static final double ROAD_SNAP_RADIUS_M = 8;

  // {distance driven, total rotation of the car} of the last WINDOW_M and a little more.
  private final ArrayDeque<double[]> mHistory = new ArrayDeque<>();
  private double mOdometerM;
  private double mYawDeg;

  /**
   * Forgets the way driven: the car was moved to another road, or a turn at a crossing is not a bend.
   */
  public void reset()
  {
    mHistory.clear();
  }

  /**
   * @param yawDeltaDeg the clockwise rotation of the car measured by the gyroscope while it drove distanceM.
   */
  public void onMotion(double yawDeltaDeg, double distanceM)
  {
    mYawDeg += yawDeltaDeg;
    mOdometerM += distanceM;
    if (!mHistory.isEmpty() && mOdometerM - mHistory.peekLast()[0] < HISTORY_STEP_M)
      return;
    mHistory.addLast(new double[] {mOdometerM, mYawDeg});
    // One point at the start of the window or before it is kept.
    while (mHistory.size() > 2 && mOdometerM - secondOdometer() >= WINDOW_M)
      mHistory.pollFirst();
  }

  private double secondOdometer()
  {
    final Iterator<double[]> it = mHistory.iterator();
    it.next();
    return it.next()[0];
  }

  /**
   * @param lat, lon the calculated position on a road.
   * @param bearing where the car looks at.
   * @param maxShiftM the largest error to look for.
   * @return how far the calculated position is ahead of the car along the road, negative if it is behind. NaN
   * if it is not known: the road is straight, or its shape doesn't tell the place clearly.
   */
  public double match(@NonNull Roads roads, double lat, double lon, double bearing, double maxShiftM)
  {
    final int count = (int) (WINDOW_M / STEP_M) + 1;
    if (mHistory.isEmpty() || mOdometerM - mHistory.peekFirst()[0] < WINDOW_M)
      return Double.NaN;

    // The rotation of the car at every step back from now.
    final double[] car = new double[count];
    double min = Double.MAX_VALUE;
    double max = -Double.MAX_VALUE;
    for (int i = 0; i < count; i++)
    {
      car[i] = yawAt(mOdometerM - i * STEP_M);
      min = Math.min(min, car[i]);
      max = Math.max(max, car[i]);
    }
    if (max - min < MIN_BEND_DEG)
      return Double.NaN;

    // The direction of the road at every step from maxShiftM ahead of the position to the end of the window
    // behind the car, were the position ahead of the car by maxShiftM.
    final int shiftSteps = (int) Math.ceil(maxShiftM / STEP_M);
    final double[] ahead = walk(roads, lat, lon, bearing, shiftSteps, true);
    final double[] behind = walk(roads, lat, lon, bearing, count - 1 + shiftSteps, false);
    if (ahead == null || behind == null)
      return Double.NaN;
    // road[shiftSteps] is the position, larger indexes are behind it.
    final double[] road = new double[ahead.length + behind.length - 1];
    for (int i = 0; i < ahead.length; i++)
      road[shiftSteps - i] = ahead[i];
    for (int i = 1; i < behind.length; i++)
      road[shiftSteps + i] = behind[i];

    double best = Double.MAX_VALUE;
    double bestShift = Double.NaN;
    final int shifts = (int) Math.round(maxShiftM / SHIFT_STEP_M);
    final double[] diffs = new double[2 * shifts + 1];
    for (int s = -shifts; s <= shifts; s++)
    {
      diffs[s + shifts] = difference(car, road, shiftSteps + s * SHIFT_STEP_M / STEP_M);
      if (diffs[s + shifts] < best)
      {
        best = diffs[s + shifts];
        bestShift = s * SHIFT_STEP_M;
      }
    }
    if (best > MAX_DIFF_DEG || Math.abs(bestShift) >= maxShiftM)
      return Double.NaN;
    final int aside = (int) Math.round(CLEAR_ASIDE_M / SHIFT_STEP_M);
    final int bestIndex = (int) Math.round(bestShift / SHIFT_STEP_M) + shifts;
    for (int i = 0; i < diffs.length; i++)
    {
      if (Math.abs(i - bestIndex) >= aside && diffs[i] < CLEAR_RATIO * Math.max(best, MIN_ASIDE_DIFF_DEG))
        return Double.NaN;
    }
    return bestShift;
  }

  private double yawAt(double odometerM)
  {
    double[] previous = null;
    for (double[] point : mHistory)
    {
      if (point[0] >= odometerM)
      {
        if (previous == null || point[0] == previous[0])
          return point[1];
        return previous[1] + (point[1] - previous[1]) * (odometerM - previous[0]) / (point[0] - previous[0]);
      }
      previous = point;
    }
    return previous != null ? previous[1] : 0;
  }

  /**
   * @return the directions of the road the car drives along at every step from the position, forward or back,
   * continuous: without jumps at 360 degrees. Null if the road ends or turns sharply earlier.
   */
  private static double[] walk(@NonNull Roads roads, double lat, double lon, double bearing, int steps,
                               boolean forward)
  {
    final double[] start = roads.snap(lat, lon, bearing, ROAD_SNAP_RADIUS_M);
    if (start == null)
      return null;
    final double[] result = new double[steps + 1];
    double posLat = start[0];
    double posLon = start[1];
    // Where the car looks at, also when walking back.
    double carBearing = RoadWalker.orient(start[2], bearing);
    result[0] = carBearing;
    for (int i = 1; i <= steps; i++)
    {
      final double moveBearing = forward ? carBearing : carBearing + 180;
      final double[] next = DeadReckoning.move(posLat, posLon, moveBearing, STEP_M);
      final double[] road = roads.snap(next[0], next[1], moveBearing, ROAD_SNAP_RADIUS_M);
      if (road == null)
        return null;
      final double roadBearing = RoadWalker.orient(road[2], carBearing);
      final double bend = DeadReckoning.angleDiff(roadBearing, carBearing);
      if (Math.abs(bend) > MAX_ROAD_BEND_DEG)
        return null;
      posLat = road[0];
      posLon = road[1];
      carBearing = roadBearing;
      result[i] = result[i - 1] + bend;
    }
    return result;
  }

  /**
   * @param roadStart the index in the road of the place the car is at, may be fractional.
   * @return how different the rotation of the car and the shape of the road are, degrees. A constant
   * difference doesn't count: the heading of the car may be wrong by some degrees.
   */
  private static double difference(@NonNull double[] car, @NonNull double[] road, double roadStart)
  {
    double sum = 0;
    double sumSquares = 0;
    for (int i = 0; i < car.length; i++)
    {
      final double position = roadStart + i;
      final int index = (int) Math.floor(position);
      final double fraction = position - index;
      final double roadBearing =
          index + 1 < road.length ? road[index] + (road[index + 1] - road[index]) * fraction : road[road.length - 1];
      final double diff = car[i] - roadBearing;
      sum += diff;
      sumSquares += diff * diff;
    }
    final double mean = sum / car.length;
    return Math.sqrt(Math.max(0, sumSquares / car.length - mean * mean));
  }
}
