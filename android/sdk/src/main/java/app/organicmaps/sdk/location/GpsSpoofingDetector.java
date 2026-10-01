package app.organicmaps.sdk.location;

import java.util.ArrayList;
import java.util.List;

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
  // A network position is sometimes kilometers away, e.g. from a Wi-Fi point that has moved or a cell tower with a wrong
  // location in the database. Such a cell tower gives the same point again and again, so a jump is used only when
  // a network position at another point near it confirms it, or when nothing contradicts it for this long.
  static final long JUMP_CONFIRM_MS = NETWORK_MAX_AGE_MS;
  // Positions from the same cell tower or Wi-Fi point.
  static final double SAME_POINT_M = 10;
  // Points that jumped away and back, they are ignored until the app restarts.
  static final int MAX_PHANTOMS = 16;
  // A Wi-Fi position (tens of meters) is used at once after a coarse cell tower one (hundreds of meters).
  static final double MORE_ACCURATE_FACTOR = 2;

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

  // The first network position of a jump away from the last trusted one, waiting for a confirmation.
  private boolean mHasJump;
  private double mJumpLat;
  private double mJumpLon;
  private long mJumpTimeMs;
  private final List<double[]> mPhantoms = new ArrayList<>();
  // Several providers report the same network position.
  private long mLastNetworkTimeMs = -1;
  private boolean mLastNetworkAccepted;

  public boolean isSpoofed()
  {
    return mSpoofed;
  }

  /**
   * @param timeMs monotonic time of the position, e.g. elapsed realtime.
   * @return false if the position jumped away and is ignored.
   */
  public boolean onNetworkPosition(double lat, double lon, double accuracyM, long timeMs)
  {
    if (timeMs == mLastNetworkTimeMs)
      return mLastNetworkAccepted;
    mLastNetworkTimeMs = timeMs;
    mLastNetworkAccepted = acceptNetworkPosition(lat, lon, accuracyM, timeMs);
    if (mLastNetworkAccepted)
    {
      mHasNetwork = true;
      mNetworkLat = lat;
      mNetworkLon = lon;
      mNetworkAccuracy = accuracyM;
      mNetworkTimeMs = timeMs;
    }
    return mLastNetworkAccepted;
  }

  private boolean acceptNetworkPosition(double lat, double lon, double accuracyM, long timeMs)
  {
    for (double[] phantom : mPhantoms)
    {
      if (distance(lat, lon, phantom[0], phantom[1]) <= SAME_POINT_M)
        return false;
    }

    // The same point again proves nothing, while the time since the last trusted position makes any jump look possible.
    if (mHasJump && distance(lat, lon, mJumpLat, mJumpLon) <= SAME_POINT_M)
    {
      if (timeMs - mJumpTimeMs < JUMP_CONFIRM_MS)
        return false;
      mHasJump = false;
      return true;
    }

    // The most recent trusted position: satellite or network.
    final boolean trustedIsLatest = mHasTrusted && (!mHasNetwork || mTrustedTimeMs >= mNetworkTimeMs);
    if (!trustedIsLatest && !mHasNetwork)
      return true;
    final double refLat = trustedIsLatest ? mTrustedLat : mNetworkLat;
    final double refLon = trustedIsLatest ? mTrustedLon : mNetworkLon;
    final long refTimeMs = trustedIsLatest ? mTrustedTimeMs : mNetworkTimeMs;
    final double refAccuracyM = trustedIsLatest ? 0 : mNetworkAccuracy;

    if (distance(lat, lon, refLat, refLon)
        <= networkTolerance(Math.max(accuracyM, refAccuracyM), timeMs - refTimeMs))
    {
      // Back from a jump soon: the jump point is wrong.
      if (mHasJump && timeMs - mJumpTimeMs < JUMP_CONFIRM_MS && mPhantoms.size() < MAX_PHANTOMS)
        mPhantoms.add(new double[] {mJumpLat, mJumpLon});
      mHasJump = false;
      return true;
    }

    if (accuracyM * MORE_ACCURATE_FACTOR < refAccuracyM)
    {
      mHasJump = false;
      return true;
    }

    if (!mHasJump)
    {
      mHasJump = true;
      mJumpLat = lat;
      mJumpLon = lon;
      mJumpTimeMs = timeMs;
      return false;
    }

    final boolean confirmed =
        distance(lat, lon, mJumpLat, mJumpLon) <= networkTolerance(accuracyM, timeMs - mJumpTimeMs);
    if (confirmed || timeMs - mJumpTimeMs >= JUMP_CONFIRM_MS)
    {
      mHasJump = false;
      return true;
    }
    // Another jump, but the time since the first one counts.
    mJumpLat = lat;
    mJumpLon = lon;
    return false;
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
      return distance(lat, lon, mNetworkLat, mNetworkLon)
          <= networkTolerance(mNetworkAccuracy, timeMs - mNetworkTimeMs);
    }

    if (mHasTrusted)
    {
      final double tolerance = MIN_JUMP_M + MAX_SPEED_MPS * Math.abs(timeMs - mTrustedTimeMs) / 1000.0;
      return distance(lat, lon, mTrustedLat, mTrustedLon) <= tolerance;
    }

    // Nothing to compare with.
    return !mSpoofed;
  }

  private static double networkTolerance(double networkAccuracyM, long dtMs)
  {
    return Math.max(NETWORK_MIN_TOLERANCE_M, NETWORK_ACCURACY_FACTOR * networkAccuracyM)
         + MAX_SPEED_MPS * Math.abs(dtMs) / 1000.0;
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
