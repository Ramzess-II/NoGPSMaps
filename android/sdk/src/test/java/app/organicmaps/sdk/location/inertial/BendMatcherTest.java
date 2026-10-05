package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class BendMatcherTest
{
  private static final double LAT = 50.45;
  private static final double LON = 30.52;
  private static final double M_PER_DEG_LAT = 111_320;
  private static final double M_PER_DEG_LON = 111_320 * Math.cos(Math.toRadians(LAT));

  /**
   * One road given by its points, {east, north} in meters.
   */
  private static final class Road implements Roads
  {
    private final List<double[]> mPoints = new ArrayList<>();
    // The direction the road is being drawn in.
    private double mBearing;

    Road(double startBearing)
    {
      mPoints.add(new double[] {0, 0});
      mBearing = startBearing;
    }

    /**
     * Continues the road turning it evenly by turnDeg, clockwise is positive.
     */
    Road add(double lengthM, double turnDeg)
    {
      final int steps = (int) Math.round(lengthM);
      for (int i = 0; i < steps; i++)
      {
        mBearing += turnDeg / steps;
        final double[] last = mPoints.get(mPoints.size() - 1);
        mPoints.add(new double[] {last[0] + Math.sin(Math.toRadians(mBearing)),
                                  last[1] + Math.cos(Math.toRadians(mBearing))});
      }
      return this;
    }

    /**
     * @return {latitude, longitude, bearing} of the road at the distance from its start.
     */
    double[] at(double distanceM)
    {
      final int index = (int) Math.round(distanceM);
      final double[] point = mPoints.get(index);
      final double[] next = mPoints.get(index + 1);
      final double bearing = Math.toDegrees(Math.atan2(next[0] - point[0], next[1] - point[1]));
      return new double[] {LAT + point[1] / M_PER_DEG_LAT, LON + point[0] / M_PER_DEG_LON, bearing};
    }

    @Nullable
    @Override
    public double[] snap(double lat, double lon, double bearing, double radius)
    {
      final double x = (lon - LON) * M_PER_DEG_LON;
      final double y = (lat - LAT) * M_PER_DEG_LAT;
      int best = -1;
      double bestDistance = radius;
      for (int i = 0; i + 1 < mPoints.size(); i++)
      {
        final double distance = Math.hypot(mPoints.get(i)[0] - x, mPoints.get(i)[1] - y);
        if (distance <= bestDistance)
        {
          bestDistance = distance;
          best = i;
        }
      }
      return best < 0 ? null : at(best);
    }

    @NonNull
    @Override
    public double[] findCrossings(double lat, double lon, double radius)
    {
      return new double[0];
    }
  }

  /**
   * Drives the car along the road from its start to the distance.
   * @param laneChangeAtM where the car changes a lane: it turns by 5 degrees and back within 40 m. NaN for none.
   */
  private static BendMatcher drive(@NonNull Road road, double distanceM, double laneChangeAtM)
  {
    final BendMatcher matcher = new BendMatcher();
    double bearing = road.at(0)[2];
    for (int m = 1; m <= distanceM; m++)
    {
      final double roadBearing = road.at(m)[2];
      double yaw = DeadReckoning.angleDiff(roadBearing, bearing);
      bearing = roadBearing;
      if (m > laneChangeAtM && m <= laneChangeAtM + 10)
        yaw += 0.5;
      else if (m > laneChangeAtM + 30 && m <= laneChangeAtM + 40)
        yaw -= 0.5;
      matcher.onMotion(yaw, 1);
    }
    return matcher;
  }

  private static double match(@NonNull BendMatcher matcher, @NonNull Road road, double positionM)
  {
    final double[] position = road.at(positionM);
    return matcher.match(road, position[0], position[1], position[2], 60);
  }

  // 300 m east, a bend of 40 degrees to the right 80 m long, 300 m straight again.
  private static Road bentRoad()
  {
    return new Road(90).add(300, 0).add(80, 40).add(300, 0);
  }

  @Test
  public void findsPositionAheadOfCar()
  {
    final Road road = bentRoad();
    // The car is 60 m after the bend, the calculated position is 25 m farther.
    final BendMatcher matcher = drive(road, 440, Double.NaN);
    assertEquals(25, match(matcher, road, 465), 3);
  }

  @Test
  public void findsPositionBehindCar()
  {
    final Road road = bentRoad();
    final BendMatcher matcher = drive(road, 440, Double.NaN);
    assertEquals(-30, match(matcher, road, 410), 3);
  }

  @Test
  public void fitsRightPosition()
  {
    final Road road = bentRoad();
    final BendMatcher matcher = drive(road, 440, Double.NaN);
    assertEquals(0, match(matcher, road, 440), 3);
  }

  @Test
  public void tellsNothingOnStraightRoad()
  {
    final Road road = new Road(90).add(600, 0);
    final BendMatcher matcher = drive(road, 400, Double.NaN);
    assertTrue(Double.isNaN(match(matcher, road, 420)));
  }

  @Test
  public void laneChangeIsNotBend()
  {
    // A lane is changed on a straight road: the car turns by 5 degrees and back.
    final Road straight = new Road(90).add(600, 0);
    assertTrue(Double.isNaN(match(drive(straight, 400, 300), straight, 420)));
    // A lane changed on the way doesn't move the position found by the bend.
    final Road road = bentRoad();
    assertEquals(25, match(drive(road, 440, 395), road, 465), 5);
  }

  @Test
  public void tellsNothingOnEvenBend()
  {
    // A roundabout or a long even bend looks the same wherever the car is on it.
    final Road road = new Road(90).add(100, 0).add(500, 300);
    final BendMatcher matcher = drive(road, 400, Double.NaN);
    assertTrue(Double.isNaN(match(matcher, road, 420)));
  }

  @Test
  public void tellsNothingBeforeEnoughIsDriven()
  {
    final Road road = bentRoad();
    final BendMatcher matcher = new BendMatcher();
    matcher.onMotion(0, 50);
    assertTrue(Double.isNaN(match(matcher, road, 400)));
  }

  @Test
  public void tellsNothingWhenErrorIsLarger()
  {
    final Road road = bentRoad();
    // The position is 100 m ahead: the bend is not within the searched 60 m, the road around looks straight.
    final BendMatcher matcher = drive(road, 440, Double.NaN);
    assertTrue(Double.isNaN(match(matcher, road, 540)));
  }
}
