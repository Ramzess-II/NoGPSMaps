package app.organicmaps.sdk.location;

/**
 * Decides when GPS can be used again after the manual mode, which the user turns on when GPS is jammed or
 * spoofed. Not being spoofed is not enough: a spoofer can move the position a little only, and the spoofing
 * detector compares it with the coarse cell tower positions. GPS is trusted when it has given accurate
 * positions for a while without a break or a jump, and they agree with the position known without GPS: the
 * mark set by the user or the inertial one.
 * <p>
 * The own position can be wrong too, e.g. a mark set in a wrong place. GPS far from it is trusted when it
 * follows the car for a long drive: it moves along the roads as fast as the car does. A spoofer can't move
 * the position along with the car.
 */
public class GpsReturnDetector
{
  // Spoofing and jamming come and go, GPS must work for a while before the manual mode is left.
  static final long MIN_STREAK_MS = 15_000;
  // GPS gives a position every second, a longer pause is a break of the streak.
  static final long MAX_GAP_MS = 3000;
  static final double MAX_ACCURACY_M = 25;
  // Faster than any car on Ukrainian roads, used to estimate how far the car could move.
  static final double MAX_SPEED_MPS = 180 / 3.6;
  // GPS and the own position are compared with this margin in addition to their errors.
  static final double MIN_TOLERANCE_M = 100;
  // GPS follows the car when it agrees with the car speed for so long and so far.
  static final long MIN_FOLLOW_MS = 30_000;
  static final double MIN_FOLLOW_DISTANCE_M = 200;
  static final double MAX_SPEED_DIFF_MPS = 2;
  static final double MAX_SPEED_DIFF_RATIO = 0.2;

  private long mStreakStartMs;
  private long mFollowStartMs;
  private double mFollowDistanceM;
  private long mLastTimeMs;
  private double mLastLat;
  private double mLastLon;
  private double mLastAccuracyM;
  private boolean mHasLast;

  public void reset()
  {
    mHasLast = false;
  }

  /**
   * Checks a GPS position trusted by the spoofing detector.
   * @param timeMs monotonic time of the position, e.g. elapsed realtime.
   * @param ownLat, ownLon the position known without GPS.
   * @param ownErrorM how far the car can be from the own position, NaN if there is no own position.
   * @param gpsSpeedMps the speed told by GPS, NaN if it is unknown.
   * @param carSpeedMps the speed of the car from its sensors, NaN if it is unknown.
   * @param onRoad true if the position is on a road.
   * @return true if GPS can be used again.
   */
  public boolean onGpsPosition(double lat, double lon, double accuracyM, long timeMs, double ownLat, double ownLon,
                               double ownErrorM, double gpsSpeedMps, double carSpeedMps, boolean onRoad)
  {
    // Positions from several providers come together, a rough one tells nothing, and GPS is lost if only rough
    // ones come: the gap breaks the streak then.
    if (accuracyM <= 0 || accuracyM > MAX_ACCURACY_M)
      return false;

    final long gapMs = timeMs - mLastTimeMs;
    final boolean continues = mHasLast && gapMs >= 0 && gapMs <= MAX_GAP_MS
                           && GpsSpoofingDetector.distance(lat, lon, mLastLat, mLastLon)
                                  <= MAX_SPEED_MPS * gapMs / 1000.0 + accuracyM + mLastAccuracyM;
    if (!continues)
    {
      mStreakStartMs = timeMs;
      mFollowStartMs = timeMs;
      mFollowDistanceM = 0;
    }
    else
    {
      mFollowDistanceM += GpsSpoofingDetector.distance(lat, lon, mLastLat, mLastLon);
    }
    if (!onRoad || !isSameSpeed(gpsSpeedMps, carSpeedMps))
    {
      mFollowStartMs = timeMs;
      mFollowDistanceM = 0;
    }

    mHasLast = true;
    mLastTimeMs = timeMs;
    mLastLat = lat;
    mLastLon = lon;
    mLastAccuracyM = accuracyM;

    if (timeMs - mFollowStartMs >= MIN_FOLLOW_MS && mFollowDistanceM >= MIN_FOLLOW_DISTANCE_M)
      return true;
    if (!Double.isNaN(ownErrorM)
        && GpsSpoofingDetector.distance(lat, lon, ownLat, ownLon) > MIN_TOLERANCE_M + accuracyM + ownErrorM)
    {
      mStreakStartMs = timeMs;
      return false;
    }
    return timeMs - mStreakStartMs >= MIN_STREAK_MS;
  }

  private static boolean isSameSpeed(double gpsSpeedMps, double carSpeedMps)
  {
    if (Double.isNaN(gpsSpeedMps) || Double.isNaN(carSpeedMps))
      return false;
    return Math.abs(gpsSpeedMps - carSpeedMps) <= Math.max(MAX_SPEED_DIFF_MPS, MAX_SPEED_DIFF_RATIO * carSpeedMps);
  }
}
