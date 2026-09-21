package app.organicmaps.sdk.location;

/**
 * Detects jammed/spoofed GNSS positions (e.g. electronic warfare moves the phone to another continent).
 * <p>
 * A satellite position is not trusted when it contradicts the network (cell towers and Wi-Fi) position,
 * or when it jumps farther than any car could drive since the last trusted satellite position.
 * After a detection, satellite positions are trusted again only after several consecutive consistent ones.
 */
public class GpsSpoofingDetector
{
  // Network positions older than this are not used to verify satellite positions.
  static final long NETWORK_MAX_AGE_MS = 3 * 60 * 1000;
  // Network positions are coarse (up to several km in the countryside), so smaller mismatches are ignored.
  static final double NETWORK_MIN_TOLERANCE_M = 3000;
  static final double NETWORK_ACCURACY_FACTOR = 3;
  // Faster than any car on Ukrainian roads, used to estimate how far the user could move between positions.
  static final double MAX_SPEED_MPS = 300 / 3.6;
  // Jumps shorter than this are GPS noise rather than spoofing.
  static final double MIN_JUMP_M = 1000;
  static final int CONSISTENT_POSITIONS_TO_RECOVER = 5;
  // Without the network only the last trusted position is available, so be more careful.
  static final int CONSISTENT_POSITIONS_TO_RECOVER_WITHOUT_NETWORK = 10;

  private static final double EARTH_RADIUS_M = 6_371_000;

  private boolean mHasNetwork;
  private double mNetworkLat;
  private double mNetworkLon;
  private double mNetworkAccuracy;
  private long mNetworkTimeMs;

  private boolean mHasTrusted;
  private double mTrustedLat;
  private double mTrustedLon;
  private long mTrustedTimeMs;

  private boolean mSpoofed;
  private int mConsistentCount;

  public boolean isSpoofed()
  {
    return mSpoofed;
  }

  /**
   * @param timeMs monotonic time of the position, e.g. elapsed realtime.
   */
  public void onNetworkPosition(double lat, double lon, double accuracyM, long timeMs)
  {
    mHasNetwork = true;
    mNetworkLat = lat;
    mNetworkLon = lon;
    mNetworkAccuracy = accuracyM;
    mNetworkTimeMs = timeMs;
  }

  /**
   * Checks a satellite (or fused) position.
   * @param timeMs monotonic time of the position, e.g. elapsed realtime.
   * @return true if the position can be trusted.
   */
  public boolean checkSatellitePosition(double lat, double lon, long timeMs)
  {
    final boolean consistent = isConsistent(lat, lon, timeMs);
    if (!mSpoofed)
    {
      if (consistent)
      {
        setTrusted(lat, lon, timeMs);
        return true;
      }
      mSpoofed = true;
      mConsistentCount = 0;
      return false;
    }

    mConsistentCount = consistent ? mConsistentCount + 1 : 0;
    final int required = hasFreshNetwork(timeMs) ? CONSISTENT_POSITIONS_TO_RECOVER
                                                 : CONSISTENT_POSITIONS_TO_RECOVER_WITHOUT_NETWORK;
    if (mConsistentCount < required)
      return false;

    mSpoofed = false;
    setTrusted(lat, lon, timeMs);
    return true;
  }

  private boolean isConsistent(double lat, double lon, long timeMs)
  {
    if (hasFreshNetwork(timeMs))
    {
      final double tolerance = Math.max(NETWORK_MIN_TOLERANCE_M, NETWORK_ACCURACY_FACTOR * mNetworkAccuracy)
                             + MAX_SPEED_MPS * Math.abs(timeMs - mNetworkTimeMs) / 1000.0;
      return distance(lat, lon, mNetworkLat, mNetworkLon) <= tolerance;
    }

    if (mHasTrusted)
    {
      final double tolerance = MIN_JUMP_M + MAX_SPEED_MPS * Math.abs(timeMs - mTrustedTimeMs) / 1000.0;
      return distance(lat, lon, mTrustedLat, mTrustedLon) <= tolerance;
    }

    // Nothing to compare with.
    return !mSpoofed;
  }

  private boolean hasFreshNetwork(long timeMs)
  {
    return mHasNetwork && Math.abs(timeMs - mNetworkTimeMs) <= NETWORK_MAX_AGE_MS;
  }

  private void setTrusted(double lat, double lon, long timeMs)
  {
    mHasTrusted = true;
    mTrustedLat = lat;
    mTrustedLon = lon;
    mTrustedTimeMs = timeMs;
  }

  static double distance(double lat1, double lon1, double lat2, double lon2)
  {
    final double dLat = Math.toRadians(lat2 - lat1);
    final double dLon = Math.toRadians(lon2 - lon1);
    final double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                   + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.sin(dLon / 2)
                         * Math.sin(dLon / 2);
    return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1, Math.sqrt(a)));
  }
}
