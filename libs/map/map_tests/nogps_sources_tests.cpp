#include "testing/testing.hpp"

#include "map/map_tests/nogps_test_env.hpp"

#include "map/nogps/elm327_session.hpp"
#include "map/nogps/esp32_source.hpp"
#include "map/nogps/phone_motion.hpp"

#include "base/math.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace nogps_sources_tests
{
using namespace nogps;
using namespace nogps_test;

class SpeedRecorder : public Elm327Session::Listener
{
public:
  void OnSpeed(int speedKmh, int64_t) override { m_speeds.push_back(speedKmh); }

  std::vector<int> m_speeds;
};

class MotionRecorder : public MotionSource::Listener
{
public:
  void OnSpeed(int speedKmh, int64_t) override { m_speeds.push_back(speedKmh); }
  void OnMotion(double yawDeltaDeg, double dtSec, int64_t) override
  {
    m_yawDeg += yawDeltaDeg;
    m_dtSec += dtSec;
    ++m_motions;
  }

  std::vector<int> m_speeds;
  double m_yawDeg = 0;
  double m_dtSec = 0;
  int m_motions = 0;
};

// Answers the initialization of the adapter.
void InitElm327(TestClock & clock, TestDelegate & delegate, Elm327Session & session)
{
  session.OnConnected();
  for (char const * command : {"ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"})
  {
    TEST_EQUAL(delegate.m_elm327Writes.back(), std::string(command) + "\r", ());
    clock.Advance(50);
    session.OnBytes("OK\r\r>");
  }
}

UNIT_TEST(NoGps_Elm327Session_InitializesAndPollsSpeed)
{
  TestClock clock;
  TestDelegate delegate;
  SpeedRecorder listener;
  Elm327Session session(delegate, clock, clock, "AA:BB", listener);
  session.Start();
  TEST_EQUAL(delegate.m_elm327Connects, std::vector<std::string>{"AA:BB"}, ());
  TEST_EQUAL(session.GetState(), SourceState::Connecting, ());

  InitElm327(clock, delegate, session);
  TEST_EQUAL(session.GetState(), SourceState::NoCarData, ());
  TEST_EQUAL(delegate.m_elm327Writes.back(), "010D\r", ());

  // The response comes in pieces.
  session.OnBytes("SEARCHING...\r41 0D 3C\r\r");
  TEST(listener.m_speeds.empty(), ());
  session.OnBytes(">");
  TEST_EQUAL(listener.m_speeds, std::vector<int>{60}, ());
  TEST_EQUAL(session.GetState(), SourceState::Connected, ());

  // The next request waits a little not to flood the car bus.
  size_t const writes = delegate.m_elm327Writes.size();
  clock.Advance(Elm327Session::kRequestIntervalMs - 1);
  TEST_EQUAL(delegate.m_elm327Writes.size(), writes, ());
  clock.Advance(1);
  TEST_EQUAL(delegate.m_elm327Writes.size(), writes + 1, ());
  TEST_EQUAL(delegate.m_elm327Writes.back(), "010D\r", ());
}

UNIT_TEST(NoGps_Elm327Session_ReconnectsWithoutCarData)
{
  TestClock clock;
  TestDelegate delegate;
  SpeedRecorder listener;
  Elm327Session session(delegate, clock, clock, "AA:BB", listener);
  session.Start();
  InitElm327(clock, delegate, session);
  session.OnBytes("41 0D 10\r\r>");
  TEST_EQUAL(session.GetState(), SourceState::Connected, ());

  // The ignition is off: the car doesn't answer, the adapter is reset after several requests.
  for (int i = 0; i < Elm327Session::kMaxNoDataRequests; ++i)
  {
    clock.Advance(Elm327Session::kRequestIntervalMs);
    TEST_EQUAL(delegate.m_elm327Writes.back(), "010D\r", ());
    session.OnBytes("NO DATA\r\r>");
    TEST_EQUAL(delegate.m_elm327Closes, i + 1 == Elm327Session::kMaxNoDataRequests ? 1 : 0, ());
    if (i + 1 < Elm327Session::kMaxNoDataRequests)
      clock.Advance(Elm327Session::kReconnectDelayMs - Elm327Session::kRequestIntervalMs);
  }
  TEST_EQUAL(session.GetState(), SourceState::Disconnected, ());
  clock.Advance(Elm327Session::kReconnectDelayMs);
  TEST_EQUAL(delegate.m_elm327Connects.size(), 2, ());
  TEST_EQUAL(session.GetState(), SourceState::Connecting, ());
}

UNIT_TEST(NoGps_Elm327Session_NoAdapterOnTimeout)
{
  TestClock clock;
  TestDelegate delegate;
  SpeedRecorder listener;
  Elm327Session session(delegate, clock, clock, "AA:BB", listener);
  session.Start();
  session.OnConnected();
  // The adapter doesn't answer the reset.
  clock.Advance(Elm327Session::kResponseTimeoutMs);
  TEST_EQUAL(delegate.m_elm327Closes, 1, ());
  TEST_EQUAL(session.GetState(), SourceState::NoAdapter, ());

  // While the adapter doesn't answer, the reconnections don't blink with "connecting".
  clock.Advance(Elm327Session::kReconnectDelayMs);
  TEST_EQUAL(delegate.m_elm327Connects.size(), 2, ());
  TEST_EQUAL(session.GetState(), SourceState::NoAdapter, ());
  session.OnClosed("Connection refused");
  TEST_EQUAL(session.GetState(), SourceState::NoAdapter, ());
  clock.Advance(Elm327Session::kReconnectDelayMs);
  TEST_EQUAL(delegate.m_elm327Connects.size(), 3, ());

  session.Stop();
  TEST_EQUAL(session.GetState(), SourceState::Disconnected, ());
  clock.Advance(10 * Elm327Session::kReconnectDelayMs);
  TEST_EQUAL(delegate.m_elm327Connects.size(), 3, ());
}

int constexpr kCalibratedFlags = esp32::kFlagImuOk | esp32::kFlagBiasOk | esp32::kFlagUpOk | esp32::kFlagObdOk;

UNIT_TEST(NoGps_Esp32Source_SendsHelloAndReadsData)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  Esp32Source source(delegate, clock, clock, "192.168.4.1", listener);
  source.Start();
  TEST(delegate.m_esp32Open, ());
  TEST_EQUAL(delegate.m_esp32Host, "192.168.4.1", ());
  TEST_EQUAL(delegate.m_esp32Port, Esp32Source::kBoxPort, ());
  TEST_EQUAL(delegate.m_esp32Sent, std::vector<std::string>{esp32::Command(1, "HELLO")}, ());
  clock.Advance(Esp32Source::kHelloIntervalMs);
  TEST_EQUAL(delegate.m_esp32Sent.back(), esp32::Command(2, "HELLO"), ());
  TEST_EQUAL(source.GetState(), SourceState::Connecting, ());

  source.OnDatagram(BoxDataLine(1, 5000, 0, 36, kCalibratedFlags));
  clock.Advance(20);
  source.OnDatagram(BoxDataLine(2, 5020, 500, 36, kCalibratedFlags));
  TEST_EQUAL(listener.m_speeds, std::vector<int>({36, 36}), ());
  TEST_ALMOST_EQUAL_ABS(listener.m_yawDeg, 0.5, 1e-9, ());
  TEST_ALMOST_EQUAL_ABS(listener.m_dtSec, 0.02, 1e-9, ());
  TEST_EQUAL(source.GetState(), SourceState::Connected, ());
  TEST(source.IsCalibrated(), ());
  TEST_EQUAL(source.GetCalibrationState(), CalibrationState::Done, ());

  // A lost line loses nothing: the box sends the totals.
  clock.Advance(40);
  source.OnDatagram(BoxDataLine(4, 5060, 1500, 36, kCalibratedFlags));
  TEST_ALMOST_EQUAL_ABS(listener.m_yawDeg, 1.5, 1e-9, ());
  TEST_ALMOST_EQUAL_ABS(listener.m_dtSec, 0.06, 1e-9, ());

  // A broken line is ignored.
  source.OnDatagram("$NGD,1,5,5080,1500,0,0,36,0,17*0000");
  TEST_EQUAL(listener.m_motions, 2, ());

  clock.Advance(Esp32Source::kDataTimeoutMs + 1);
  TEST_EQUAL(source.GetState(), SourceState::Connecting, ());
  TEST(!source.IsCalibrated(), ());
}

UNIT_TEST(NoGps_Esp32Source_ReadsLinesOfBluetoothStream)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  Esp32Source source(delegate, clock, clock, "192.168.4.1", listener);
  source.Start();
  TEST(delegate.m_esp32BleOpen, ());
  source.OnBleState(BleState::Connected);

  // Two lines in a piece.
  source.OnBleBytes(BoxDataLine(1, 5000, 0, 36, kCalibratedFlags) + BoxDataLine(2, 5020, 500, 36, kCalibratedFlags));
  TEST_EQUAL(listener.m_speeds, std::vector<int>({36, 36}), ());
  TEST_ALMOST_EQUAL_ABS(listener.m_yawDeg, 0.5, 1e-9, ());
  TEST_EQUAL(source.GetLink(), Esp32Link::Ble, ());
  TEST_EQUAL(source.GetState(), SourceState::Connected, ());

  // A line in three pieces.
  std::string const line = BoxDataLine(3, 5040, 1000, 37, kCalibratedFlags);
  source.OnBleBytes(line.substr(0, 7));
  source.OnBleBytes(line.substr(7, 20));
  TEST_EQUAL(listener.m_speeds.size(), 2, ());
  source.OnBleBytes(line.substr(27));
  TEST_EQUAL(listener.m_speeds.back(), 37, ());
  TEST_ALMOST_EQUAL_ABS(listener.m_yawDeg, 1.0, 1e-9, ());

  // The end of a line whose start was lost, a line cut in the middle and a whole one after them.
  source.OnBleBytes("234,0,0,36,0,17*ABCD\r\n" + BoxDataLine(4, 5060, 1500, 38, kCalibratedFlags).substr(0, 12) +
                    BoxDataLine(5, 5080, 2000, 39, kCalibratedFlags));
  TEST_EQUAL(listener.m_speeds.back(), 39, ());
  TEST_EQUAL(listener.m_speeds.size(), 4, ());
  TEST_ALMOST_EQUAL_ABS(listener.m_yawDeg, 2.0, 1e-9, ());

  source.Stop();
  TEST(!delegate.m_esp32BleOpen, ());
  TEST_EQUAL(source.GetLink(), Esp32Link::None, ());
}

UNIT_TEST(NoGps_Esp32Source_CallsBoxOverBluetoothWhileItWorks)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  Esp32Source source(delegate, clock, clock, "192.168.4.1", listener);
  source.Start();
  // No Bluetooth yet: the box is called over Wi-Fi only.
  source.OnBleState(BleState::Searching);
  clock.Advance(Esp32Source::kHelloIntervalMs);
  TEST_EQUAL(delegate.m_esp32Sent.size(), 2, ());
  TEST(delegate.m_esp32BleSent.empty(), ());
  source.OnDatagram(BoxDataLine(1, 5000, 0, 36, kCalibratedFlags));
  TEST_EQUAL(source.GetLink(), Esp32Link::Wifi, ());

  // Bluetooth is connected: the box is called over it at once, and not over Wi-Fi, so it sends its data there.
  source.OnBleState(BleState::Connected);
  TEST_EQUAL(delegate.m_esp32BleSent.size(), 1, ());
  TEST_EQUAL(delegate.m_esp32BleSent.back(), esp32::Command(3, "HELLO"), ());
  TEST_EQUAL(delegate.m_esp32Sent.size(), 2, ());
  int64_t boxTimeMs = 5020;
  for (int i = 0; i < 5; ++i)
  {
    clock.Advance(Esp32Source::kHelloIntervalMs);
    source.OnBleBytes(BoxDataLine(2 + i, boxTimeMs += 1000, 0, 36, kCalibratedFlags));
  }
  TEST_EQUAL(delegate.m_esp32BleSent.size(), 6, ());
  TEST_EQUAL(delegate.m_esp32Sent.size(), 2, ());
  TEST_EQUAL(source.GetLink(), Esp32Link::Ble, ());

  // Bluetooth is connected but silent, e.g. the box has forgotten the phone: Wi-Fi is tried too.
  clock.Advance(Esp32Source::kBleSilenceMs + Esp32Source::kHelloIntervalMs);
  TEST_GREATER(delegate.m_esp32Sent.size(), 2, ());
  TEST_EQUAL(delegate.m_esp32Sent.back(), delegate.m_esp32BleSent.back(), ());
  TEST_EQUAL(source.GetLink(), Esp32Link::None, ());

  // Bluetooth is lost: Wi-Fi only.
  source.OnBleState(BleState::Searching);
  size_t const bleSent = delegate.m_esp32BleSent.size();
  clock.Advance(Esp32Source::kHelloIntervalMs);
  TEST_EQUAL(delegate.m_esp32BleSent.size(), bleSent, ());
}

UNIT_TEST(NoGps_Esp32Source_CalibratesAndTimesOut)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  Esp32Source source(delegate, clock, clock, "192.168.4.1", listener);
  source.Start();
  source.OnDatagram(BoxDataLine(1, 5000, 0, 0, kCalibratedFlags));

  source.Calibrate();
  TEST_EQUAL(delegate.m_esp32Sent.back(), esp32::Command(1, "CAL_UP"), ());
  TEST_EQUAL(source.GetCalibrationState(), CalibrationState::Calibrating, ());
  // A reply to another command doesn't count.
  source.OnDatagram(BoxLine("NGA,7,OK"));
  TEST_EQUAL(source.GetCalibrationState(), CalibrationState::Calibrating, ());
  source.OnDatagram(BoxLine("NGA,1,PROGRESS,40"));
  TEST_EQUAL(source.GetCalibrationProgressPercent(), 40, ());
  source.OnDatagram(BoxLine("NGA,1,OK"));
  TEST_EQUAL(source.GetCalibrationState(), CalibrationState::Done, ());

  source.Calibrate();
  TEST_EQUAL(delegate.m_esp32Sent.back(), esp32::Command(2, "CAL_UP"), ());
  source.OnDatagram(BoxLine("NGA,2,ERR,MOVING"));
  TEST_EQUAL(source.GetCalibrationState(), CalibrationState::FailedMoving, ());

  // The box doesn't answer.
  source.Calibrate();
  clock.Advance(Esp32Source::kCalibrationTimeoutMs + 1);
  TEST_EQUAL(source.GetCalibrationState(), CalibrationState::FailedMoving, ());
}

UNIT_TEST(NoGps_Esp32Source_RequestsLostEvents)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  Esp32Source source(delegate, clock, clock, "192.168.4.1", listener);
  source.Start();
  auto const sent = [&delegate] { return delegate.m_esp32Sent.size(); };

  // The events before the connection are asked from the journal of the box.
  size_t count = sent();
  source.OnDatagram(BoxLine("NGE,1,5,1000,W,OBD_LOST,no answer"));
  TEST_EQUAL(sent(), count + 1, ());
  TEST_EQUAL(delegate.m_esp32Sent.back(), esp32::Command(1, "EVENTS,0"), ());
  // The next event in a row asks nothing.
  count = sent();
  source.OnDatagram(BoxLine("NGE,1,6,1100,I,OBD_OK,"));
  TEST_EQUAL(sent(), count, ());
  // A gap is asked not more often than once in a while.
  source.OnDatagram(BoxLine("NGE,1,8,1200,I,CAL_AUTO,-386"));
  TEST_EQUAL(sent(), count, ());
  clock.Advance(Esp32Source::kEventsRequestIntervalMs);
  source.OnDatagram(BoxLine("NGE,1,10,1300,I,CAL_AUTO,-386"));
  TEST_EQUAL(delegate.m_esp32Sent.back(), esp32::Command(2, "EVENTS,9"), ());
  // The status tells the last event of the box.
  clock.Advance(Esp32Source::kEventsRequestIntervalMs);
  source.OnDatagram(BoxLine("NGS,1,0.2.0,ICM20602,500,OK,,12,331,0,OK,0x12,NONE,38400,-386,12"));
  TEST_EQUAL(delegate.m_esp32Sent.back(), esp32::Command(3, "EVENTS,11"), ());
}

UNIT_TEST(NoGps_Esp32Source_TellsSleepingBox)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  Esp32Source source(delegate, clock, clock, "192.168.4.1", listener);
  source.Start();
  source.OnDatagram(BoxDataLine(1, 5000, 0, 0, esp32::kFlagImuOk));
  TEST_EQUAL(source.GetState(), SourceState::NoCarData, ());
  source.OnDatagram(BoxLine("NGS,1,0.2.0,ICM20602,500,NO_ADAPTER,,,331,0,OK,0x12,NO_ADAPTER,0,-386,4,,,"));
  TEST_EQUAL(source.GetState(), SourceState::NoAdapter, ());
  // The adapter is searched again: it is still missing.
  source.OnDatagram(BoxLine("NGS,1,0.2.0,ICM20602,500,INIT,,,331,0,OK,0x12,NONE,0,-386,4,,,"));
  TEST_EQUAL(source.GetState(), SourceState::NoAdapter, ());
  source.OnDatagram(BoxLine("NGS,1,0.2.0,ICM20602,500,SEARCHING,,,331,0,OK,0x12,NONE,0,-386,4,,,"));
  TEST_EQUAL(source.GetState(), SourceState::ObdConnecting, ());

  // The engine is stopped: the box switches its Wi-Fi off later, it is not lost.
  source.OnDatagram(BoxLine("NGS,1,0.2.0,ICM20602,500,OK,,12,331,0,OK,0x12,NONE,38400,-386,14,OFF,,12480"));
  TEST(source.GetCarInfo(), ());
  TEST_EQUAL(source.GetCarInfo()->m_engineRunning, false, ());
  clock.Advance(Esp32Source::kDataTimeoutMs + 1);
  TEST_EQUAL(source.GetState(), SourceState::BoxSleeping, ());
  TEST(!source.GetCarInfo(), ());

  source.Stop();
  TEST(!delegate.m_esp32Open, ());
  TEST_EQUAL(source.GetState(), SourceState::Disconnected, ());
}

UNIT_TEST(NoGps_PhoneMotion_CalibratesAndTurns)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  PhoneMotion phone(delegate, clock, clock, {} /* elm327Address */, listener);
  phone.Start();
  TEST_EQUAL(phone.GetState(), SourceState::Disconnected, ());
  // The phone lies flat screen up.
  phone.OnAccel({0, 0, 9.81});
  Vec3 const bias = {0.001, -0.002, 0.003};

  phone.Calibrate();
  for (int i = 0; i <= GyroCalibrator::kRequiredSamples; ++i)
  {
    clock.Advance(20);
    phone.OnGyro(clock.NowMs() * 1'000'000, bias);
  }
  TEST_EQUAL(phone.GetCalibrationState(), CalibrationState::Done, ());
  TEST(phone.IsCalibrated(), ());

  // Turning right at 10 deg/s is a negative rotation around the screen normal.
  double const yawBefore = listener.m_yawDeg;
  for (int i = 0; i < 50; ++i)
  {
    clock.Advance(20);
    phone.OnGyro(clock.NowMs() * 1'000'000, {bias[0], bias[1], bias[2] - math::DegToRad(10.0)});
  }
  TEST_ALMOST_EQUAL_ABS(listener.m_yawDeg - yawBefore, 10.0, 1e-6, ());

  // A break of the sensor is not a rotation.
  clock.Advance(1000);
  int const motions = listener.m_motions;
  phone.OnGyro(clock.NowMs() * 1'000'000, bias);
  TEST_EQUAL(listener.m_motions, motions, ());
}

UNIT_TEST(NoGps_PhoneMotion_CalibrationFailsWhileDriving)
{
  TestClock clock;
  TestDelegate delegate;
  MotionRecorder listener;
  PhoneMotion phone(delegate, clock, clock, std::string("AA:BB"), listener);
  phone.Start();
  TEST_EQUAL(delegate.m_elm327Connects, std::vector<std::string>{"AA:BB"}, ());
  phone.OnAccel({0, 9.81, 0});

  // The car drives straight: the phone is as still as in a standing car, but the speed tells it moves.
  auto * elm327 = phone.GetElm327();
  TEST(elm327, ());
  InitElm327(clock, delegate, *elm327);
  elm327->OnBytes("41 0D 28\r\r>");
  phone.Calibrate();
  for (int i = 0; i <= GyroCalibrator::kRequiredSamples; ++i)
  {
    clock.Advance(20);
    phone.OnGyro(clock.NowMs() * 1'000'000, {0, 0, 0});
  }
  TEST_EQUAL(phone.GetCalibrationState(), CalibrationState::FailedMoving, ());
  TEST(!phone.IsCalibrated(), ());
}
}  // namespace nogps_sources_tests
