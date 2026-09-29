package app.organicmaps.sdk.location.inertial;

/**
 * Dead reckoning: moves the position by the car speed along the heading.
 * <p>
 * The accuracy degrades with the distance driven since the last reference position (GPS, a manual mark),
 * mostly because of the heading error.
 */
public class DeadReckoning
{
  static final double BASE_ACCURACY_M = 15;
  static final double ACCURACY_PER_METER = 0.05;
  // A road going in a different direction is not the road the car is on (or the car is turning).
  static final double MAX_SNAP_HEADING_DIFF_DEG = 30;
  // The heading is pulled to the road slowly and only when they nearly agree: it removes the slow gyroscope
  // drift only. A larger difference is a real turn or a road going away from the car, e.g. a fork or a part of a
  // roundabout behind the car, where the gyroscope is right and the road is not.
  static final double MAX_HEADING_PULL_DIFF_DEG = 10;
  static final double SNAP_HEADING_WEIGHT = 0.1;

  private static final double EARTH_RADIUS_M = 6_371_000;

  private boolean mHasPosition;
  private boolean mHasHeading;
  private double mLat;
  private double mLon;
  private double mHeadingDeg;
  private double mSpeedMps;
  private double mDistanceSinceFixM;

  public void setPosition(double lat, double lon)
  {
    mHasPosition = true;
    mLat = lat;
    mLon = lon;
    mDistanceSinceFixM = 0;
  }

  /**
   * Moves the position without making it a reference one: the accuracy keeps degrading.
   */
  public void moveTo(double lat, double lon)
  {
    mLat = lat;
    mLon = lon;
  }

  /**
   * @param headingDeg clockwise from the north.
   */
  public void setHeading(double headingDeg)
  {
    mHasHeading = true;
    mHeadingDeg = normalize(headingDeg);
  }

  public void setSpeed(double speedMps)
  {
    mSpeedMps = Math.max(0, speedMps);
  }

  /**
   * @param deltaDeg clockwise rotation.
   */
  public void rotate(double deltaDeg)
  {
    if (mHasHeading)
      mHeadingDeg = normalize(mHeadingDeg + deltaDeg);
  }

  /**
   * Moves the position along the heading with the current speed.
   */
  public void advance(double dtSec)
  {
    if (!isReady() || dtSec <= 0)
      return;

    final double distance = mSpeedMps * dtSec;
    final double[] moved = move(mLat, mLon, mHeadingDeg, distance);
    mLat = moved[0];
    mLon = moved[1];
    mDistanceSinceFixM += distance;
  }

  /**
   * Pulls the position to the road the car is on: it removes the side error. The heading is corrected
   * towards the road direction, it removes the gyroscope drift on straight roads.
   * @param roadBearingDeg direction of the road, the opposite direction is the same road.
   * @return false if the road goes in another direction and it is ignored.
   */
  public boolean snapToRoad(double lat, double lon, double roadBearingDeg)
  {
    if (!isReady())
      return false;
    double diff = angleDiff(roadBearingDeg, mHeadingDeg);
    if (Math.abs(diff) > 90)
      diff = angleDiff(roadBearingDeg + 180, mHeadingDeg);
    if (Math.abs(diff) > MAX_SNAP_HEADING_DIFF_DEG)
      return false;
    mLat = lat;
    mLon = lon;
    if (Math.abs(diff) <= MAX_HEADING_PULL_DIFF_DEG)
      mHeadingDeg = normalize(mHeadingDeg + diff * SNAP_HEADING_WEIGHT);
    return true;
  }

  /**
   * @return {latitude, longitude} of the point at the distance in the bearing direction.
   */
  public static double[] move(double lat, double lon, double bearingDeg, double distanceM)
  {
    final double bearing = Math.toRadians(bearingDeg);
    final double movedLat = lat + Math.toDegrees(distanceM * Math.cos(bearing) / EARTH_RADIUS_M);
    final double movedLon =
        lon + Math.toDegrees(distanceM * Math.sin(bearing) / (EARTH_RADIUS_M * Math.cos(Math.toRadians(movedLat))));
    return new double[] {movedLat, movedLon};
  }

  /**
   * @return the shortest signed rotation from {@code from} to {@code to}, in [-180, 180).
   */
  public static double angleDiff(double to, double from)
  {
    return normalize(to - from + 180) - 180;
  }

  public boolean hasPosition()
  {
    return mHasPosition;
  }

  public boolean hasHeading()
  {
    return mHasHeading;
  }

  public boolean isReady()
  {
    return mHasPosition && mHasHeading;
  }

  public double getLat()
  {
    return mLat;
  }

  public double getLon()
  {
    return mLon;
  }

  public double getHeading()
  {
    return mHeadingDeg;
  }

  public double getSpeed()
  {
    return mSpeedMps;
  }

  public double getDistanceSinceFix()
  {
    return mDistanceSinceFixM;
  }

  public double getAccuracy()
  {
    return BASE_ACCURACY_M + ACCURACY_PER_METER * mDistanceSinceFixM;
  }

  public static double normalize(double deg)
  {
    final double result = deg % 360;
    return result < 0 ? result + 360 : result;
  }
}
