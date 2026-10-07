#pragma once

#include "map/nogps/accel_log.hpp"
#include "map/nogps/esp32_protocol.hpp"
#include "map/nogps/motion_source.hpp"
#include "map/nogps/scheduler.hpp"

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <string_view>

namespace nogps
{
class Clock;
class Delegate;

/// The NoGPS ESP32 sensor box: it is fixed in the car and sends the car rotation and speed over its own Wi-Fi
/// network, the phone can be held in hands. The box zeroes its gyroscope by itself on every stop.
class Esp32Source : public MotionSource
{
public:
  static uint16_t constexpr kBoxPort = 4210;
  // The box sends data while it hears the phone.
  static int64_t constexpr kHelloIntervalMs = 1000;
  // The box sends 50 lines a second, a longer silence means it is lost.
  static int64_t constexpr kDataTimeoutMs = 1000;
  // A calibration takes ~3 s on the box.
  static int64_t constexpr kCalibrationTimeoutMs = 10'000;
  // Events lost over Wi-Fi are asked again not more often than this.
  static int64_t constexpr kEventsRequestIntervalMs = 2000;
  // More than the box keeps in its journal.
  static size_t constexpr kMaxLoggedEvents = 64;

  /// \param address the address of the box, 192.168.4.1 for its own access point.
  Esp32Source(Delegate & delegate, Scheduler & scheduler, Clock const & clock, std::string address,
              MotionSource::Listener & listener);
  ~Esp32Source() override;

  // MotionSource overrides:
  void Start() override;
  void Stop() override;
  SourceState GetState() const override;
  bool IsCalibrated() const override;
  CalibrationState GetCalibrationState() override;
  int GetCalibrationProgressPercent() const override { return m_calibrationProgress; }
  void Calibrate() override;
  std::string GetDeviceName() const override;
  std::optional<CarInfo> GetCarInfo() const override;

  void OnDatagram(std::string_view text);

private:
  void SendHello();
  void Send(std::string_view command, int id);
  void OnData(esp32::Data const & data);
  void OnEvent(esp32::Event const & event);
  void OnReply(esp32::Reply const & reply);
  void OnLastEventNumber(int64_t lastNumber);
  void RequestLostEvents(int64_t fromNumber);
  bool IsConnected() const;
  bool IsSleeping() const;

  Delegate & m_delegate;
  Clock const & m_clock;
  std::string const m_address;
  MotionSource::Listener & m_listener;
  Timer m_helloTimer;
  bool m_running = false;
  int m_helloId = 0;

  // Nothing until the first data line.
  std::optional<int64_t> m_lastDataMs;
  int m_flags = 0;
  AccelLog m_accelLog;
  bool m_hasLast = false;
  int64_t m_lastTimeMs = 0;
  int64_t m_lastYawMdeg = 0;
  std::string m_imuName;
  // The state of the ELM327 adapter of the box from its status.
  std::optional<std::string> m_obdState;
  // The box hasn't found its ELM327: while it searches it again, the adapter is still missing.
  bool m_obdAdapterMissing = false;
  // The engine, its speed and the voltage of the car from the status of the box.
  std::optional<CarInfo> m_carInfo;
  // The box has told that its voltage and the one of ELM327 differ. Kept until they come close again.
  bool m_voltageMismatch = false;
  // The numbers of the events of the box written to the log.
  std::set<int64_t> m_loggedEvents;
  std::optional<int64_t> m_eventsRequestMs;
  int m_nextCommandId = 1;
  // The calibration command waiting for its reply, 0 if none.
  int m_calibrationId = 0;
  int64_t m_calibrationStartMs = 0;
  int m_calibrationProgress = 0;
  bool m_calibrationFailed = false;
};
}  // namespace nogps
