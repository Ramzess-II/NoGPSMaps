#include "map/nogps/service.hpp"

#include "map/routing_manager.hpp"

#include "routing/following_info.hpp"

#include "platform/location.hpp"

namespace nogps
{
namespace
{
// The core takes a negative bearing for an unknown one: it is built with fast math, which does not support NaN.
double ToCoreBearing(std::optional<double> bearingDeg)
{
  return bearingDeg.value_or(-1.0);
}
}  // namespace

std::optional<RoadPoint> RoutingMapApi::RoutingRoads::Snap(ms::LatLon const & point, std::optional<double> bearingDeg,
                                                           double radiusM)
{
  RoadPoint road;
  if (!m_routing.SnapToRoad(point, ToCoreBearing(bearingDeg), radiusM, m_matchRoute, road.m_point, road.m_bearingDeg))
    return {};
  return road;
}

std::vector<ms::LatLon> RoutingMapApi::RoutingRoads::FindCrossings(ms::LatLon const & center, double radiusM)
{
  return m_routing.FindRoadCrossings(center, radiusM);
}

RoutingMapApi::RoutingMapApi(RoutingManager & routing, Clock const & clock, ShowCarHeadingFn && showCarHeading)
  : m_routing(routing)
  , m_clock(clock)
  , m_showCarHeading(std::move(showCarHeading))
  , m_roads(routing, false /* matchRoute */)
  , m_routeRoads(routing, true /* matchRoute */)
{}

bool RoutingMapApi::IsNavigating() const
{
  return m_routing.IsRoutingActive() && m_routing.IsRoutingFollowing();
}

void RoutingMapApi::RebuildRouteIfOffRoute(Fix const & fix, double offRouteDistanceM, std::optional<double> bearingDeg)
{
  location::GpsInfo info;
  info.m_source = location::EUser;
  info.m_timestamp = m_clock.UnixNowMs() / 1000.0;
  info.m_latitude = fix.m_position.m_lat;
  info.m_longitude = fix.m_position.m_lon;
  info.m_horizontalAccuracy = offRouteDistanceM;
  info.m_bearing = ToCoreBearing(bearingDeg);
  m_routing.RebuildRouteIfOffRoute(info);
}

std::optional<RoadPoint> RoutingMapApi::ProjectToRoute(ms::LatLon const & position, double radiusM)
{
  RoadPoint road;
  if (!m_routing.ProjectToRoute(position, radiusM, road.m_point, road.m_bearingDeg))
    return {};
  return road;
}

std::optional<RoadWalker::Result> RoutingMapApi::ShiftAlongRoute(ms::LatLon const & position,
                                                                 std::optional<double> bearingDeg, double distanceM)
{
  RoadWalker::Result result;
  if (!m_routing.ShiftAlongRoute(position, ToCoreBearing(bearingDeg), distanceM, result.m_position, result.m_bearingDeg,
                                 result.m_appliedM, result.m_atCrossing))
  {
    return {};
  }
  return result;
}

std::optional<RoadPoint> RoutingMapApi::SnapToMainRoad(ms::LatLon const & position, double radiusM)
{
  RoadPoint road;
  if (!m_routing.SnapToMainRoad(position, radiusM, road.m_point, road.m_bearingDeg))
    return {};
  return road;
}

std::string RoutingMapApi::GetDistanceLeft() const
{
  if (!m_routing.IsRoutingActive())
    return {};
  routing::FollowingInfo info;
  m_routing.GetRouteFollowingInfo(info);
  if (!info.IsValid())
    return {};
  return info.m_distToTarget.GetDistanceString() + info.m_distToTarget.GetUnitsString();
}

Service::Service(Delegate & delegate, RoutingManager & routing, RoutingMapApi::ShowCarHeadingFn && showCarHeading)
  : m_map(routing, m_clock, std::move(showCarHeading))
  , m_engine(delegate, m_map, m_storage, m_clock, m_scheduler)
{}
}  // namespace nogps
