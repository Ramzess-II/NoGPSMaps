#include "map/nogps/turn_matcher.hpp"

#include "map/nogps/geo.hpp"
#include "map/nogps/road_walker.hpp"

#include "base/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nogps
{
namespace
{
// Returns true if a road leaves the crossing in the direction.
bool HasRoadFromCrossing(Roads & roads, ms::LatLon const & crossing, double bearingDeg)
{
  auto const point = Move(crossing, bearingDeg, TurnMatcher::kRoadCheckDistanceM);
  auto const road = roads.Snap(point, bearingDeg, TurnMatcher::kRoadCheckRadiusM);
  if (!road || Distance(point, road->m_point) > TurnMatcher::kRoadCheckRadiusM)
    return false;
  return std::fabs(AngleDiff(Orient(road->m_bearingDeg, bearingDeg), bearingDeg)) <= TurnMatcher::kMaxRoadDiffDeg;
}
}  // namespace

std::optional<ms::LatLon> TurnMatcher::FindCrossing(Roads & roads, ms::LatLon const & corner, double fromBearingDeg,
                                                    double toBearingDeg, double searchM)
{
  double const turn = std::fabs(AngleDiff(toBearingDeg, fromBearingDeg));
  if (turn < kMinTurnDeg || turn > kMaxTurnDeg)
    return {};

  double const radius = std::max(kMinSearchM, std::min(kMaxSearchM, searchM));
  // A bend of the road with a side street close to it would move the car to the street.
  if (IsBend(roads, corner, fromBearingDeg, toBearingDeg, radius))
    return {};

  std::optional<ms::LatLon> best;
  double bestAlong = std::numeric_limits<double>::max();
  for (auto const & crossing : roads.FindCrossings(corner, radius))
  {
    // The offset of the crossing along the road the car came by and to the side of it.
    double const distance = Distance(corner, crossing);
    double const angle = math::DegToRad(AngleDiff(Bearing(corner, crossing), fromBearingDeg));
    double const along = std::fabs(distance * std::cos(angle));
    double const side = std::fabs(distance * std::sin(angle));
    if (along > radius || side > kMaxSideOffsetM || along >= bestAlong)
      continue;
    if (!HasRoadFromCrossing(roads, crossing, fromBearingDeg + 180) ||
        !HasRoadFromCrossing(roads, crossing, toBearingDeg))
    {
      continue;
    }
    best = crossing;
    bestAlong = along;
  }
  return best;
}

bool TurnMatcher::IsBend(Roads & roads, ms::LatLon const & position, double fromBearingDeg, double toBearingDeg,
                         double distanceM)
{
  auto const start = roads.Snap(position, fromBearingDeg, RoadWalker::kSnapRadiusM);
  if (!start)
    return false;
  ms::LatLon pos = start->m_point;
  double heading = Orient(start->m_bearingDeg, fromBearingDeg);
  for (double walked = 0; walked < distanceM; walked += RoadWalker::kStepM)
  {
    if (std::fabs(AngleDiff(toBearingDeg, heading)) <= kMaxBendEndDiffDeg)
      return true;
    auto const road = roads.Snap(Move(pos, heading, RoadWalker::kStepM), heading, RoadWalker::kSnapRadiusM);
    if (!road)
      return false;
    double const roadHeading = Orient(road->m_bearingDeg, heading);
    if (std::fabs(AngleDiff(roadHeading, heading)) > RoadWalker::kMaxBendDeg)
      return false;
    pos = road->m_point;
    heading = roadHeading;
  }
  return std::fabs(AngleDiff(toBearingDeg, heading)) <= kMaxBendEndDiffDeg;
}
}  // namespace nogps
