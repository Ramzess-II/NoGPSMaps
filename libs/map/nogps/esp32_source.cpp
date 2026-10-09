#include "map/nogps/esp32_source.hpp"

#include "map/nogps/clock.hpp"
#include "map/nogps/delegate.hpp"

#include "base/logging.hpp"

namespace nogps
{
Esp32Source::Esp32Source(Delegate & delegate, Scheduler & scheduler, Clock const & clock, std::string address,
                         MotionSource::Listener & listener)
  : m_delegate(delegate)
  , m_clock(clock)
  , m_address(std::move(address))
  , m_listener(listener)
  , m_helloTimer(scheduler)
{}

Esp32Source::~Esp32Source()
{
  Stop();
}

void Esp32Source::Start()
{
  if (m_running)
    return;
  LOG(LINFO, ("address =", m_address));
  m_running = true;
  m_delegate.Esp32Open(m_address, kBoxPort);
  m_delegate.Esp32BleOpen();
  SendHello();
}

void Esp32Source::Stop()
{
  if (!m_running)
    return;
  LOG(LINFO, ("ESP32 stopped"));
  m_running = false;
  m_helloTimer.Stop();
  m_delegate.Esp32Close();
  m_delegate.Esp32BleClose();
  m_bleState = BleState::Off;
  m_bleConnectedMs.reset();
  m_lastBleLineMs.reset();
  m_bleLine.clear();
  m_link = Esp32Link::None;
  m_hasLast = false;
  m_lastDataMs.reset();
}

void Esp32Source::SendHello()
{
  Send("HELLO", ++m_helloId);
  m_helloTimer.Start(kHelloIntervalMs, [this] { SendHello(); });
}

void Esp32Source::Send(std::string_view command, int id)
{
  auto const line = esp32::Command(id, command);
  // Bluetooth is preferred: the phone doesn't have to stay in the Wi-Fi network of the box. The box sends its
  // data where the last HELLO has come from, so it is not called over Wi-Fi while Bluetooth works.
  bool const ble = m_bleState == BleState::Connected;
  if (ble)
    m_delegate.Esp32BleSend(line);
  if (!ble || IsBleSilent())
    m_delegate.Esp32Send(line);
}

bool Esp32Source::IsBleSilent() const
{
  int64_t const now = m_clock.NowMs();
  // Just connected: the box sends nothing before it gets a HELLO.
  if (m_bleConnectedMs && now - *m_bleConnectedMs <= kBleSilenceMs)
    return false;
  return !m_lastBleLineMs || now - *m_lastBleLineMs > kBleSilenceMs;
}

void Esp32Source::OnBleState(BleState state)
{
  if (!m_running || state == m_bleState)
    return;
  LOG(LINFO, ("Bluetooth =", state));
  m_bleState = state;
  m_bleLine.clear();
  if (state != BleState::Connected)
    return;
  m_bleConnectedMs = m_clock.NowMs();
  // The box starts sending at once, not at the next HELLO.
  SendHello();
}

void Esp32Source::OnBleBytes(std::string_view bytes)
{
  if (!m_running)
    return;
  for (char const c : bytes)
  {
    // A line starts with "$": what was received of a line cut in the middle is dropped.
    if (c == '$')
      m_bleLine.clear();
    else if (m_bleLine.empty())
      continue;
    if (c == '\n')
    {
      OnLine(m_bleLine, Esp32Link::Ble);
      m_bleLine.clear();
    }
    else if (m_bleLine.size() < kMaxLineSize)
    {
      m_bleLine += c;
    }
    else
    {
      m_bleLine.clear();
    }
  }
}

Esp32Link Esp32Source::GetLink() const
{
  return IsConnected() ? m_link : Esp32Link::None;
}

void Esp32Source::OnDatagram(std::string_view text)
{
  if (m_running)
    OnLine(text, Esp32Link::Wifi);
}

void Esp32Source::OnLine(std::string_view text, Esp32Link link)
{
  auto const fields = esp32::Parse(text);
  if (!fields)
    return;
  if (link == Esp32Link::Ble)
    m_lastBleLineMs = m_clock.NowMs();
  if (auto const data = esp32::ParseData(*fields))
  {
    if (link != m_link)
    {
      LOG(LINFO, ("The box is heard over", link));
      m_link = link;
    }
    OnData(*data);
    return;
  }
  if (auto const reply = esp32::ParseReply(*fields))
  {
    OnReply(*reply);
    return;
  }
  if (auto const event = esp32::ParseEvent(*fields))
  {
    OnEvent(*event);
    return;
  }
  if (auto const lastNumber = esp32::ParseLastEventNumber(*fields))
    OnLastEventNumber(*lastNumber);
  if (auto const imuName = esp32::ParseImuName(*fields))
    m_imuName = *imuName;
  if (auto const carInfo = esp32::ParseCarInfo(*fields))
  {
    if (carInfo->m_boxMillivolts >= 0 && carInfo->GetReferenceMillivolts() >= 0)
      m_voltageMismatch = carInfo->m_voltageMismatch;
    m_carInfo = carInfo;
  }
  auto const obdState = esp32::ParseObdState(*fields);
  if (obdState && obdState != m_obdState)
  {
    LOG(LINFO, ("OBD state =", *obdState));
    m_obdState = obdState;
    if (*obdState == "NO_ADAPTER")
      m_obdAdapterMissing = true;
    else if (*obdState != "INIT")
      m_obdAdapterMissing = false;
  }
}

void Esp32Source::OnData(esp32::Data const & data)
{
  int64_t const now = m_clock.NowMs();
  bool const fresh = m_lastDataMs && now - *m_lastDataMs <= kDataTimeoutMs;
  m_lastDataMs = now;
  m_flags = data.m_flags;
  if (data.m_speedKmh >= 0)
    m_listener.OnSpeed(data.m_speedKmh, now - data.m_speedAgeMs);
  // For the trip log only, see AccelLog.
  if (data.m_hasAccel)
  {
    auto const line =
        m_accelLog.OnSample(data.m_timeMs * 1'000'000, data.m_accelH1, data.m_accelH2, data.m_accelUp, data.m_jolt);
    if (line && data.m_speedKmh > 0)
      LOG(LINFO, ("ACCB", *line));
    m_listener.OnSourceAccel({data.m_accelH1, data.m_accelH2, data.m_accelUp});
  }

  // The box sends the totals: a lost line loses nothing, the next one has the rotation.
  if (m_hasLast && fresh)
  {
    // The time of the box is 32 bits and wraps around.
    int64_t const dtMs = (data.m_timeMs - m_lastTimeMs) & 0xFFFFFFFFLL;
    double const yawDeltaDeg = IsCalibrated() ? (data.m_yawMdeg - m_lastYawMdeg) / 1000.0 : 0;
    m_listener.OnMotion(yawDeltaDeg, dtMs / 1000.0, now * 1'000'000);
  }
  m_hasLast = true;
  m_lastTimeMs = data.m_timeMs;
  m_lastYawMdeg = data.m_yawMdeg;
}

void Esp32Source::OnEvent(esp32::Event const & event)
{
  int64_t const lastNumber = m_loggedEvents.empty() ? -1 : *m_loggedEvents.rbegin();
  // The journal of the box sent on request comes after the new events, so the order doesn't matter.
  if (!m_loggedEvents.insert(event.m_number).second)
    return;
  if (m_loggedEvents.size() > kMaxLoggedEvents)
    m_loggedEvents.erase(m_loggedEvents.begin());
  // The errors of the box are logged to find later why the speed or the gyroscope was lost.
  if (event.m_level == "I")
    LOG(LINFO, ("Box event #", event.m_number, "at", event.m_timeMs, "ms:", event.m_code, event.m_text));
  else
    LOG(LWARNING, ("Box event #", event.m_number, "at", event.m_timeMs, "ms:", event.m_code, event.m_text));
  // The box compares the raw voltage, the status has the corrected one.
  if (event.m_code == "VOLT_MISMATCH")
    m_voltageMismatch = true;
  else if (event.m_code == "VOLT_CAL")
    m_voltageMismatch = false;
  if (event.m_number > lastNumber + 1)
    RequestLostEvents(lastNumber + 1);
}

void Esp32Source::RequestLostEvents(int64_t fromNumber)
{
  // The events were lost over Wi-Fi or came before the connection, the journal of the box has them.
  int64_t const now = m_clock.NowMs();
  if (m_eventsRequestMs && now - *m_eventsRequestMs < kEventsRequestIntervalMs)
    return;
  m_eventsRequestMs = now;
  Send("EVENTS," + std::to_string(fromNumber), m_nextCommandId++);
}

void Esp32Source::OnLastEventNumber(int64_t lastNumber)
{
  if (lastNumber < 0)
    return;
  // The box has restarted and counts its events again.
  if (!m_loggedEvents.empty() && lastNumber < *m_loggedEvents.rbegin())
    m_loggedEvents.clear();
  int64_t const loggedNumber = m_loggedEvents.empty() ? -1 : *m_loggedEvents.rbegin();
  if (lastNumber > loggedNumber)
    RequestLostEvents(loggedNumber + 1);
}

void Esp32Source::OnReply(esp32::Reply const & reply)
{
  if (reply.m_id != m_calibrationId)
    return;
  if (reply.m_progress)
  {
    m_calibrationProgress = *reply.m_progress;
    return;
  }
  LOG(LINFO, ("Calibration: ok =", reply.m_ok, "error =", reply.m_error));
  m_calibrationId = 0;
  m_calibrationFailed = !reply.m_ok;
}

bool Esp32Source::IsConnected() const
{
  return m_running && m_lastDataMs && m_clock.NowMs() - *m_lastDataMs <= kDataTimeoutMs;
}

SourceState Esp32Source::GetState() const
{
  if (!m_running)
    return SourceState::Disconnected;
  if (!IsConnected())
    return IsSleeping() ? SourceState::BoxSleeping : SourceState::Connecting;
  if (m_flags & esp32::kFlagObdOk)
    return SourceState::Connected;
  if (m_obdState == "SLEEP")
    return SourceState::ObdSleeping;
  // OBD_ABSENT is set also when ELM327 is off in the box, only the status tells one from another.
  if (m_obdAdapterMissing)
    return SourceState::NoAdapter;
  if (!m_obdState)
    return SourceState::NoCarData;
  if (*m_obdState == "DISABLED")
    return SourceState::ObdDisabled;
  if (*m_obdState == "INIT" || *m_obdState == "SEARCHING")
    return SourceState::ObdConnecting;
  if (*m_obdState == "ERROR")
    return SourceState::ObdError;
  // NO_CAR and NO_DATA: the adapter answers, the car doesn't.
  return SourceState::NoCarData;
}

bool Esp32Source::IsSleeping() const
{
  // The box switches its Wi-Fi off a minute after the engine is stopped, so a box that has gone after it has told
  // about the stopped engine is not lost.
  if (!m_lastDataMs)
    return false;
  return m_obdState == "SLEEP" || (m_carInfo && m_carInfo->m_engineRunning == false);
}

std::optional<CarInfo> Esp32Source::GetCarInfo() const
{
  if (!IsConnected() || !m_carInfo)
    return {};
  CarInfo car = *m_carInfo;
  car.m_voltageMismatch = m_voltageMismatch;
  return car;
}

bool Esp32Source::IsCalibrated() const
{
  int constexpr kRequired = esp32::kFlagImuOk | esp32::kFlagBiasOk | esp32::kFlagUpOk;
  return IsConnected() && (m_flags & kRequired) == kRequired && (m_flags & esp32::kFlagMountMoved) == 0;
}

CalibrationState Esp32Source::GetCalibrationState()
{
  if (m_calibrationId != 0 && m_clock.NowMs() - m_calibrationStartMs > kCalibrationTimeoutMs)
  {
    LOG(LWARNING, ("No reply to the calibration"));
    m_calibrationId = 0;
    m_calibrationFailed = true;
  }
  if (m_calibrationId != 0)
    return CalibrationState::Calibrating;
  if (m_calibrationFailed)
    return CalibrationState::FailedMoving;
  if (IsConnected() && (m_flags & esp32::kFlagMountMoved))
    return CalibrationState::MountMoved;
  return IsCalibrated() ? CalibrationState::Done : CalibrationState::None;
}

void Esp32Source::Calibrate()
{
  // The box zeroes the gyroscope and finds its vertical in the car at once.
  m_calibrationId = m_nextCommandId++;
  m_calibrationStartMs = m_clock.NowMs();
  m_calibrationProgress = 0;
  m_calibrationFailed = false;
  LOG(LINFO, ("Calibration, id =", m_calibrationId));
  Send("CAL_UP", m_calibrationId);
}

std::string Esp32Source::GetDeviceName() const
{
  return m_imuName.empty() ? "ESP32" : "ESP32 · " + m_imuName;
}
}  // namespace nogps
