#include "map/nogps/delegate.hpp"

#include "base/assert.hpp"

namespace nogps
{
std::string DebugPrint(Provider provider)
{
  switch (provider)
  {
  case Provider::Gps: return "gps";
  case Provider::Network: return "network";
  case Provider::Fused: return "fused";
  case Provider::Manual: return "manual";
  case Provider::Inertial: return "inertial";
  }
  UNREACHABLE();
}

std::string DebugPrint(Event event)
{
  switch (event)
  {
  case Event::ManualModeChanged: return "ManualModeChanged";
  case Event::GpsBack: return "GpsBack";
  case Event::GpsLost: return "GpsLost";
  case Event::GpsSpoofed: return "GpsSpoofed";
  case Event::GpsRestored: return "GpsRestored";
  case Event::RoadLost: return "RoadLost";
  case Event::TurnsReversed: return "TurnsReversed";
  case Event::MotionSourceStopped: return "MotionSourceStopped";
  case Event::MarkNoRoad: return "MarkNoRoad";
  }
  UNREACHABLE();
}
}  // namespace nogps
