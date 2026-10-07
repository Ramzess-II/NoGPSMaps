#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace nogps
{
/// Packs the accelerometer for the trip log: one line a second, kSlices parts of it each with the mean
/// acceleration along the axes of the sensor and the jolt, i.e. how far the acceleration went from that mean.
/// The log is to check on real trips what the accelerometer can tell: braking before the car reports it,
/// bumps and rails as known places of the map.
class AccelLog
{
public:
  static int constexpr kSlices = 5;

  /// \param x, y, z the acceleration, m/s².
  /// \param jolt the jolt during the sample measured by the sensor itself, which reads faster.
  /// \returns the line of the second that has just ended.
  std::optional<std::string> OnSample(int64_t timestampNs, double x, double y, double z, double jolt = 0);

  void Reset();

private:
  void ClearSlice();
  void EndSlice();

  std::string m_line;
  int m_slices = 0;
  int64_t m_sliceStartNs = 0;
  int m_count = 0;
  std::array<double, 3> m_sum{};
  std::array<double, 3> m_min{};
  std::array<double, 3> m_max{};
  double m_jolt = 0;
};
}  // namespace nogps
