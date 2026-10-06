#pragma once

#include <optional>

namespace nogps
{
/// What the sensor box knows about the car besides its speed. The box asks the car for the engine speed and
/// ELM327 for the voltage only while the car stands, not to delay the speed. -1 is unknown.
struct CarInfo
{
  /// \returns the voltage the box compares its own one with: of the control unit of the car, ELM327 clones
  /// show less. -1 if unknown.
  int GetReferenceMillivolts() const { return m_ecuMillivolts >= 0 ? m_ecuMillivolts : m_elmMillivolts; }

  std::optional<bool> m_engineRunning;
  int m_rpm = -1;
  // The voltage of the car in millivolts. The box measures it by itself all the time, ELM327 and the control unit
  // of the car are asked while the car stands.
  int m_boxMillivolts = -1;
  int m_elmMillivolts = -1;
  int m_ecuMillivolts = -1;
  // The voltage of the box differs too much from the car: the voltage divider of the box is wrong.
  bool m_voltageMismatch = false;
};
}  // namespace nogps
