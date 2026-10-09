#pragma once

#include "platform/settings.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace nogps
{
/// What the navigation without GPS keeps between the starts of the app: the settings and what it has learned.
class Storage
{
public:
  // The keys are shared with the settings of the Android app made before the navigation moved to the core.
  static std::string_view constexpr kInertialEnabled = "NoGpsInertialEnabled";
  static std::string_view constexpr kElm327Address = "NoGpsElm327Address";
  static std::string_view constexpr kEsp32Source = "NoGpsEsp32Source";
  static std::string_view constexpr kEsp32Address = "NoGpsEsp32Address";
  // The ESP32 sensor box is reached over Bluetooth LE, not over its Wi-Fi network.
  static std::string_view constexpr kEsp32Bluetooth = "NoGpsEsp32Bluetooth";
  static std::string_view constexpr kShiftStepM = "NoGpsShiftStep";
  static std::string_view constexpr kShiftButtonsShown = "NoGpsShiftButtons";
  static std::string_view constexpr kSpeedScale = "NoGpsSpeedScale";
  static std::string_view constexpr kSpeedTable = "NoGpsSpeedTable";
  static std::string_view constexpr kSpeedLag = "NoGpsSpeedLag";
  static std::string_view constexpr kWrongNetworkPoints = "NoGpsWrongNetworkPoints";
  static std::string_view constexpr kLastTrusted = "NoGpsLastTrusted";

  virtual ~Storage() = default;

  virtual std::optional<std::string> GetString(std::string_view key) const = 0;
  virtual void SetString(std::string_view key, std::string const & value) = 0;

  template <class T>
  T Get(std::string_view key, T const & defaultValue) const
  {
    auto const s = GetString(key);
    T value;
    if (s && settings::FromString(*s, value))
      return value;
    return defaultValue;
  }

  template <class T>
  void Set(std::string_view key, T const & value)
  {
    SetString(key, settings::ToString(value));
  }
};

/// The storage in the settings of the app.
class SettingsStorage : public Storage
{
public:
  std::optional<std::string> GetString(std::string_view key) const override;
  void SetString(std::string_view key, std::string const & value) override;
};
}  // namespace nogps
