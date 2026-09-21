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
    final double heading = Math.toRadians(mHeadingDeg);
    mLat += Math.toDegrees(distance * Math.cos(heading) / EARTH_RADIUS_M);
    mLon += Math.toDegrees(distance * Math.sin(heading) / (EARTH_RADIUS_M * Math.cos(Math.toRadians(mLat))));
    mDistanceSinceFixM += distance;
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

  static double normalize(double deg)
  {
    final double result = deg % 360;
    return result < 0 ? result + 360 : result;
  }
}
