package app.organicmaps.sdk.location.inertial;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import org.junit.Test;

public class RoadsTest
{
  private static final double LAT = 50.45;
  private static final double LON = 30.52;
  private static final double M_PER_DEG_LAT = 111_320;
  private static final double M_PER_DEG_LON = 111_320 * Math.cos(Math.toRadians(LAT));
  // Streets go east and north every 100 m, they cross at the multiples of 100 m.
  private static final double BLOCK_M = 100;

  private static double lat(double northM)
  {
    return LAT + northM / M_PER_DEG_LAT;
  }

  private static double lon(double eastM)
  {
    return LON + eastM / M_PER_DEG_LON;
  }

  private static double east(double lon)
  {
    return (lon - LON) * M_PER_DEG_LON;
  }

  private static double north(double lat)
  {
    return (lat - LAT) * M_PER_DEG_LAT;
  }

  /**
   * A grid of streets, the part with |x| or |y| more than 500 m has no roads.
   */
  private static final Roads GRID = new Roads() {
    @Nullable
    @Override
    public double[] snap(double lat, double lon, double bearing, double radius)
    {
      final double x = east(lon);
      final double y = north(lat);
      // The closest east street and the closest north street.
      final double eastStreetY = Math.round(y / BLOCK_M) * BLOCK_M;
      final double northStreetX = Math.round(x / BLOCK_M) * BLOCK_M;
      final double toEast = Math.abs(y - eastStreetY);
      final double toNorth = Math.abs(x - northStreetX);
      boolean east = toEast <= toNorth;
      // A road going the car's way is preferred.
      if (!Double.isNaN(bearing))
      {
        final boolean looksEast = Math.abs(Math.sin(Math.toRadians(bearing))) > Math.sqrt(0.5);
        if (looksEast && toEast <= radius)
          east = true;
        else if (!looksEast && toNorth <= radius)
          east = false;
      }
      if ((east ? toEast : toNorth) > radius || Math.abs(x) > 500 || Math.abs(y) > 500)
        return null;
      return east ? new double[] {lat(eastStreetY), lon, 90} : new double[] {lat, lon(northStreetX), 0};
    }

    @NonNull
    @Override
    public double[] findCrossings(double lat, double lon, double radius)
    {
      final double x = east(lon);
      final double y = north(lat);
      final java.util.List<Double> result = new java.util.ArrayList<>();
      for (double cx = -500; cx <= 500; cx += BLOCK_M)
      {
        for (double cy = -500; cy <= 500; cy += BLOCK_M)
        {
          if (Math.abs(cx - x) <= radius && Math.abs(cy - y) <= radius)
          {
            result.add(lat(cy));
            result.add(lon(cx));
          }
        }
      }
      final double[] array = new double[result.size()];
      for (int i = 0; i < array.length; i++)
        array[i] = result.get(i);
      return array;
    }
  };

  private static void assertAt(double eastM, double northM, @NonNull double[] position)
  {
    assertEquals(eastM, east(position[1]), 0.5);
    assertEquals(northM, north(position[0]), 0.5);
  }

  @Test
  public void walksAlongRoad()
  {
    final double[] walked = RoadWalker.walk(GRID, lat(0), lon(10), 90, 50);
    assertNotNull(walked);
    assertAt(60, 0, walked);
    assertEquals(90, walked[2], 1);
    assertEquals(50, walked[3], 0.5);
    assertEquals(0, walked[4], 0);
  }

  @Test
  public void snapsToRoadAxis()
  {
    final double[] walked = RoadWalker.walk(GRID, lat(4), lon(10), 90, 20);
    assertNotNull(walked);
    assertAt(30, 0, walked);
  }

  @Test
  public void stopsAtCrossing()
  {
    final double[] walked = RoadWalker.walk(GRID, lat(0), lon(60), 90, 100);
    assertNotNull(walked);
    assertAt(100, 0, walked);
    assertEquals(40, walked[3], 0.5);
    assertEquals(1, walked[4], 0);
  }

  @Test
  public void walksBackKeepingDirection()
  {
    final double[] walked = RoadWalker.walk(GRID, lat(0), lon(60), 90, -100);
    assertNotNull(walked);
    assertAt(0, 0, walked);
    assertEquals(90, walked[2], 1);
    assertEquals(-60, walked[3], 0.5);
    assertEquals(1, walked[4], 0);
  }

  @Test
  public void leavesCrossingItStandsAt()
  {
    final double[] walked = RoadWalker.walk(GRID, lat(0), lon(100), 90, 30);
    assertNotNull(walked);
    assertAt(130, 0, walked);
    assertEquals(0, walked[4], 0);
  }

  @Test
  public void followsRoadTheCarLooksAlong()
  {
    // The car looks north on the north street, the east one crossing it is ignored.
    final double[] walked = RoadWalker.walk(GRID, lat(10), lon(0), 0, 50);
    assertNotNull(walked);
    assertAt(0, 60, walked);
    assertEquals(0, walked[2], 1);
  }

  @Test
  public void stopsAtRoadEnd()
  {
    final double[] walked = RoadWalker.walk(GRID, lat(0), lon(490), 90, 50);
    assertNotNull(walked);
    assertEquals(10, walked[3], 5.1);
  }

  @Test
  public void noRoad()
  {
    assertNull(RoadWalker.walk(GRID, lat(50), lon(50), 90, 50));
  }

  @Test
  public void orientsRoadTheCarWay()
  {
    assertEquals(270, RoadWalker.orient(90, 250), 0);
    assertEquals(90, RoadWalker.orient(90, 100), 0);
    assertEquals(90, RoadWalker.orient(270, 100), 0);
    assertEquals(90, RoadWalker.orient(90, Double.NaN), 0);
  }

  @Test
  public void matchesTurnBeforeCrossing()
  {
    // The calculated distance lags: the car turned left at 200 m, but it is at 160 m by the calculation.
    final double[] crossing = TurnMatcher.findCrossing(GRID, lat(0), lon(160), 90, 0, 20);
    assertNotNull(crossing);
    assertAt(200, 0, crossing);
  }

  @Test
  public void matchesTurnAfterCrossing()
  {
    final double[] crossing = TurnMatcher.findCrossing(GRID, lat(0), lon(240), 90, 180, 20);
    assertNotNull(crossing);
    assertAt(200, 0, crossing);
  }

  @Test
  public void ignoresBends()
  {
    assertNull(TurnMatcher.findCrossing(GRID, lat(0), lon(200), 90, 70, 20));
  }

  @Test
  public void ignoresTurnsAround()
  {
    assertNull(TurnMatcher.findCrossing(GRID, lat(0), lon(200), 90, 270, 20));
  }

  @Test
  public void ignoresParallelStreets()
  {
    // A crossing 30 m aside is on a parallel street, not on the street the car has driven along.
    assertNull(TurnMatcher.findCrossing(GRID, lat(30), lon(190), 90, 0, 20));
  }

  @Test
  public void noCrossingOutsideRoads()
  {
    assertNull(TurnMatcher.findCrossing(GRID, lat(0), lon(700), 90, 0, 20));
  }
}
