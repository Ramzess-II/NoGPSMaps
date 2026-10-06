#pragma once

#include "map/nogps/roads.hpp"

#include "geometry/latlon.hpp"

#include <optional>

namespace nogps
{
/// Finds the crossing the car has turned at. The gyroscope tells the turn exactly, but the calculated
/// distance lags behind or runs ahead of the car, so the calculated turn is before or after the real crossing
/// and the car would leave the road after it. The real crossing is the one on the road the car has driven
/// along with a road going the new way.
class TurnMatcher
{
public:
  // Smaller turns are bends of the road.
  static double constexpr kMinTurnDeg = 35;
  // Larger turns are turns around, not at a crossing.
  static double constexpr kMaxTurnDeg = 150;
  // The crossing is searched this far along the road at least, the calculated distance is often wrong by
  // tens of meters.
  static double constexpr kMinSearchM = 60;
  static double constexpr kMaxSearchM = 300;
  // A crossing farther from the road the car drove along is on a parallel street.
  static double constexpr kMaxSideOffsetM = 20;
  // The roads of a crossing are checked this far from it: close to it all its roads meet.
  static double constexpr kRoadCheckDistanceM = 12;
  static double constexpr kRoadCheckRadiusM = 6;
  static double constexpr kMaxRoadDiffDeg = 35;
  // The road the car was on going the new way within this difference has bent, the car has followed it.
  static double constexpr kMaxBendEndDiffDeg = 25;

  /// \param corner where the car has turned by the calculation.
  /// \param fromBearingDeg where the car looked before the turn.
  /// \param toBearingDeg where the car looks after the turn.
  /// \param searchM how far along the road the crossing is searched, e.g. the accuracy of the calculation.
  /// \returns the crossing, or nothing if the turn is not at a crossing.
  static std::optional<ms::LatLon> FindCrossing(Roads & roads, ms::LatLon const & corner, double fromBearingDeg,
                                                double toBearingDeg, double searchM);

  /// Follows the road the car was on from the corner: the road going on the way the car looked before. At a
  /// crossing it goes straight on or ends.
  /// \returns true if the road bends to the new direction within the distance.
  static bool IsBend(Roads & roads, ms::LatLon const & position, double fromBearingDeg, double toBearingDeg,
                     double distanceM);
};
}  // namespace nogps
