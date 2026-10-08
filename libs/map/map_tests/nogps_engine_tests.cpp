#include "testing/testing.hpp"

#include "map/map_tests/nogps_test_env.hpp"

#include "map/nogps/engine.hpp"
#include "map/nogps/esp32_protocol.hpp"
#include "map/nogps/geo.hpp"
#include "map/nogps/storage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>

namespace nogps_engine_tests
{
using namespace nogps;
using namespace nogps_test;

int constexpr kCalibratedFlags = esp32::kFlagImuOk | esp32::kFlagBiasOk | esp32::kFlagUpOk | esp32::kFlagObdOk;

class Env
{
public:
  /// \param roads the roads instead of the grid of streets.
  explicit Env(bool inertial, Roads * roads = nullptr)
  {
    // The inertial navigation takes the roads when it starts.
    m_map.m_roads = roads;
    if (inertial)
    {
      m_storage.Set(Storage::kInertialEnabled, true);
      m_storage.Set(Storage::kEsp32Source, true);
    }
    m_engine.Start();
  }

  /// The ESP32 box sends its data 50 times a second for |ms|: the car drives at |speedKmh| turning at |yawRateDegS|,
  /// clockwise is positive.
  void Drive(int64_t ms, int speedKmh, double yawRateDegS)
  {
    for (int64_t t = 0; t < ms; t += 20)
    {
      m_clock.Advance(20);
      m_boxTimeMs += 20;
      m_yawMdeg += yawRateDegS * 20;
      m_engine.OnEsp32Datagram(BoxDataLine(++m_seq, m_boxTimeMs, std::llround(m_yawMdeg), speedKmh, kCalibratedFlags));
    }
  }

  /// GPS positions every second for |seconds| at |speedMps| from the point, going east along the street.
  void DriveGps(int seconds, double eastM, double speedMps)
  {
    for (int i = 0; i < seconds; ++i)
    {
      m_clock.Advance(1000);
      m_engine.OnFix(GpsAt(m_clock, eastM + i * speedMps, 0, 90.0, speedMps));
    }
  }

  Fix const & Last() const { return m_delegate.m_positions.back(); }

  TestClock m_clock;
  TestStorage m_storage;
  TestDelegate m_delegate;
  TestMap m_map;
  Engine m_engine{m_delegate, m_map, m_storage, m_clock, m_clock};

private:
  int64_t m_seq = 0;
  int64_t m_boxTimeMs = 100'000;
  double m_yawMdeg = 0;
};

void TestAlmostEqualAbs(double actual, double expected, double eps)
{
  TEST_ALMOST_EQUAL_ABS(actual, expected, eps, ());
}

// The car is marked on the street going east, the box tells it stands.
void MarkCar(Env & env, double eastM)
{
  env.m_engine.SetManualMode(true);
  env.Drive(500, 0, 0);
  TEST(env.m_engine.PlaceMarkByTap(At(eastM, 3)), ());
  TEST_EQUAL(env.Last().m_provider, Provider::Manual, ());
  TestAlmostEqualAbs(East(env.Last().m_position), eastM, 0.5);
}

UNIT_TEST(NoGps_Engine_StartsTheBox)
{
  Env env(true /* inertial */);
  TEST(env.m_delegate.m_esp32Open, ());
  // The accelerometer goes to the trip log, the gyroscope is in the box.
  TEST(env.m_delegate.m_sensorsStarted, ());
  TEST(!env.m_delegate.m_gyroscope, ());
  env.Drive(500, 0, 0);
  auto const status = env.m_engine.GetStatus();
  TEST(status.m_inertialStarted, ());
  TEST(status.m_esp32Source, ());
  TEST_EQUAL(status.m_sourceState, SourceState::Connected, ());
  TEST_EQUAL(status.m_calibration, CalibrationState::Done, ());
  TEST_EQUAL(status.m_speedKmh, 0, ());

  env.m_engine.Stop();
  TEST(!env.m_delegate.m_esp32Open, ());
  TEST(!env.m_delegate.m_sensorsStarted, ());
}

UNIT_TEST(NoGps_Engine_MarkIsOnRoadAndShowsCarHeading)
{
  Env env(false /* inertial */);
  env.m_engine.SetManualMode(true);
  TEST(env.m_delegate.HasEvent(Event::ManualModeChanged), ());
  // A tap between the streets is not where the car is.
  TEST(!env.m_engine.PlaceMarkByTap(At(50, 50)), ());
  TEST(env.m_delegate.HasEvent(Event::MarkNoRoad), ());
  TEST(env.m_engine.PlaceMarkByTap(At(60, 4)), ());
  TestAlmostEqualAbs(North(env.Last().m_position), 0, 0.5);
  TEST_EQUAL(env.Last().m_bearingDeg, 90.0, ());
  TEST(env.m_engine.IsCarHeadingShown(), ());
  TEST_EQUAL(env.m_map.m_carHeading, 90.0, ());
  auto const status = env.m_engine.GetStatus();
  TEST_EQUAL(status.m_source, PositionSource::Manual, ());
  TEST(status.m_movedByHand, ());

  // The mark is sent again and again, otherwise the position is treated as lost.
  size_t const positions = env.m_delegate.m_positions.size();
  env.m_clock.Advance(Engine::kManualRepeatIntervalMs);
  TEST_EQUAL(env.m_delegate.m_positions.size(), positions + 1, ());

  env.m_engine.ReverseDirection();
  TEST_EQUAL(env.Last().m_bearingDeg, 270.0, ());
}

UNIT_TEST(NoGps_Engine_ShiftStopsAtCrossing)
{
  Env env(false /* inertial */);
  env.m_engine.SetManualMode(true);
  TEST(env.m_engine.PlaceMarkByTap(At(60, 2)), ());
  // There is no route, the position is moved along the road up to the crossing.
  TestAlmostEqualAbs(env.m_engine.ShiftPosition(100), 40, 0.5);
  TestAlmostEqualAbs(East(env.Last().m_position), 100, 0.5);
  // The car may turn at the crossing: it is not moved further that way, but it is moved back.
  TEST(env.m_engine.IsShiftBlocked(true /* forward */), ());
  TEST(!env.m_engine.IsShiftBlocked(false /* forward */), ());
  TEST_EQUAL(env.m_engine.ShiftPosition(50), 0, ());
  TestAlmostEqualAbs(env.m_engine.ShiftPosition(-30), -30, 0.5);
  TestAlmostEqualAbs(East(env.Last().m_position), 70, 0.5);
  TEST(!env.m_engine.IsShiftBlocked(true /* forward */), ());
}

UNIT_TEST(NoGps_Engine_CyclesShiftSteps)
{
  Env env(false /* inertial */);
  TEST_EQUAL(env.m_engine.GetStatus().m_shiftStepM, 50, ());
  env.m_engine.CycleShiftStep();
  TEST_EQUAL(env.m_engine.GetStatus().m_shiftStepM, 100, ());
  env.m_engine.CycleShiftStep();
  TEST_EQUAL(env.m_engine.GetStatus().m_shiftStepM, 10, ());
}

UNIT_TEST(NoGps_Engine_InertialTurnIsMovedToCrossing)
{
  Env env(true /* inertial */);
  MarkCar(env, 50);
  // 10 m/s east, the calculated position lags behind the car: the car turns left at the crossing at 200 m while it
  // is at ~160 m by the calculation.
  env.Drive(10'000, 36, 0);
  TEST_EQUAL(env.Last().m_provider, Provider::Inertial, ());
  double const beforeTurnM = East(env.Last().m_position);
  TEST_LESS(beforeTurnM, 180, ());
  TestAlmostEqualAbs(North(env.Last().m_position), 0, 1);
  env.Drive(3000, 36, -30);
  env.Drive(3000, 36, 0);
  // The car is on the street going north from the crossing.
  TestAlmostEqualAbs(East(env.Last().m_position), 200, 1);
  TEST_GREATER(North(env.Last().m_position), 20, ());
  TestAlmostEqualAbs(AngleDiff(*env.Last().m_bearingDeg, 0), 0, 3);
  TEST(!env.m_delegate.HasEvent(Event::RoadLost), ());
}

UNIT_TEST(NoGps_Engine_TurnAlongBranchIsNotMovedToNextCrossing)
{
  // A road branches off the street going east to the north-east at 100 m, and a street goes the same way from
  // the crossing at 115 m, 13 m aside of the branch. The branch leaves the street not at a crossing of the map.
  double const sin30 = 0.5;
  double const cos30 = std::sqrt(0.75);
  PieceRoads roads({{-300, 0, 500, 0},
                    {100, 0, 100 + 300 * sin30, 300 * cos30},
                    {115, 0, 115 + 300 * sin30, 300 * cos30}},
                   {{115, 0}});
  Env env(true /* inertial */, &roads);
  MarkCar(env, 0);
  // 10 m/s east, then the car takes the branch, 60 degrees to the left: the turn of 19 m radius starts 11 m
  // before the branch.
  env.Drive(7700, 36, 0);
  TEST_EQUAL(env.Last().m_provider, Provider::Inertial, ());
  TestAlmostEqualAbs(East(env.Last().m_position), 89, 3);
  env.Drive(2000, 36, -30);
  env.Drive(4000, 36, 0);
  // The car is on the branch, not on the street going the same way from the crossing.
  auto const & position = env.Last().m_position;
  double const fromBranchM = (East(position) - 100) * cos30 - North(position) * sin30;
  TestAlmostEqualAbs(fromBranchM, 0, 3);
  TEST_GREATER(North(position), 30, ());
  TEST(!env.m_delegate.HasEvent(Event::RoadLost), ());
}

UNIT_TEST(NoGps_Engine_StopsCarThatLeftRoads)
{
  Env env(true /* inertial */);
  MarkCar(env, 50);
  // The heading goes wrong slowly, slower than a turn: the car leaves the road.
  env.Drive(12'000, 36, -5);
  env.Drive(5000, 36, 0);
  TEST(env.m_delegate.HasEvent(Event::RoadLost), ());
  // The car stands where it was on the road the last time until the user marks it.
  auto const stopped = env.Last().m_position;
  env.Drive(2000, 36, 0);
  TestAlmostEqualAbs(Distance(env.Last().m_position, stopped), 0, 0.01);
  TEST(env.m_engine.PlaceMarkByTap(At(300, 0)), ());
  env.Drive(1000, 36, 0);
  TEST_GREATER(East(env.Last().m_position), 300, ());
}

UNIT_TEST(NoGps_Engine_GpsIsBackAfterRoadIsLost)
{
  Env env(true /* inertial */);
  MarkCar(env, -400);
  // The heading goes wrong slowly, slower than a turn: the car leaves the road, the calculation stops.
  env.Drive(12'000, 36, -5);
  env.Drive(5000, 36, 0);
  TEST(env.m_delegate.HasEvent(Event::RoadLost), ());
  TEST(env.m_engine.IsManualMode(), ());
  // GPS works again, far from where the calculation has stopped: there is nothing to compare it with, it
  // is trusted after it has worked for long enough.
  for (int i = 0; i <= GpsReturnDetector::kMinStreakMs / 1000 + 1; ++i)
  {
    env.Drive(1000, 36, 0);
    env.m_engine.OnFix(GpsAt(env.m_clock, 10 * i, 300, 90.0, 10.0));
  }
  TEST(!env.m_engine.IsManualMode(), ());
  TEST(env.m_delegate.HasEvent(Event::GpsBack), ());
  TEST_EQUAL(env.Last().m_provider, Provider::Gps, ());
}

UNIT_TEST(NoGps_Engine_TripLineHasPlatformPositions)
{
  Env env(true /* inertial */);
  TEST(env.m_engine.GetTripLine().find(" gps=") == std::string::npos, ());

  env.m_clock.Advance(1000);
  env.m_engine.OnFix(GpsAt(env.m_clock, 100, 0, 90.0, 10.0, 4));
  env.m_clock.Advance(250);
  char expected[128];
  std::snprintf(expected, sizeof(expected), " gps=%.6f,%.6f/4.0/36.0/90.0/0.25 ", At(100, 0).m_lat, At(100, 0).m_lon);
  std::string line = env.m_engine.GetTripLine();
  TEST(line.find(expected) != std::string::npos, (line));

  // The position GPS is ignored with, e.g. to test the inertial navigation, is written too: the shown position is
  // compared with it. This one has no speed and no bearing.
  env.m_engine.SetGpsDisabled(true);
  env.m_clock.Advance(1000);
  env.m_engine.OnFix(GpsAt(env.m_clock, 200, 0));
  std::snprintf(expected, sizeof(expected), " gps=%.6f,%.6f/5.0/-/-/0.00 ", At(200, 0).m_lat, At(200, 0).m_lon);
  line = env.m_engine.GetTripLine();
  TEST(line.find(expected) != std::string::npos, (line));

  // The satellites mixed with Wi-Fi and cell towers, the only position of iOS, are written apart.
  TEST(line.find(" fused=") == std::string::npos, (line));
  auto fused = GpsAt(env.m_clock, 300, 0, 90.0, 10.0, 8);
  fused.m_provider = Provider::Fused;
  env.m_engine.OnFix(fused);
  std::snprintf(expected, sizeof(expected), " fused=%.6f,%.6f/8.0/36.0/90.0/0.00 ", At(300, 0).m_lat, At(300, 0).m_lon);
  line = env.m_engine.GetTripLine();
  TEST(line.find(expected) != std::string::npos, (line));
  TEST(line.find(" gps=") != std::string::npos, (line));

  // An old position tells nothing about the car now.
  env.m_clock.Advance(Engine::kGpsStatusMaxAgeMs + 1000);
  line = env.m_engine.GetTripLine();
  TEST(line.find(" gps=") == std::string::npos && line.find(" fused=") == std::string::npos, (line));
}

UNIT_TEST(NoGps_Engine_PauseResumesByItself)
{
  Env env(true /* inertial */);
  MarkCar(env, 50);
  env.Drive(2000, 36, 0);
  env.m_engine.TogglePause();
  TEST(env.m_engine.GetStatus().m_paused, ());
  // The car maneuvers: the speed is positive, but the position stays. The positions are sent 5 times a second, the
  // first one after the pause is where the car has stopped.
  env.Drive(2 * InertialNavigator::kOutputIntervalMs, 10, 0);
  double const pausedAtM = East(env.Last().m_position);
  env.Drive(2000, 10, 0);
  TestAlmostEqualAbs(East(env.Last().m_position), pausedAtM, 0.01);
  // Nobody drives so fast backwards: the pause is over.
  env.Drive(3500, 20, 0);
  TEST(!env.m_engine.GetStatus().m_paused, ());
  env.Drive(1000, 20, 0);
  TEST_GREATER(East(env.Last().m_position), pausedAtM, ());
}

UNIT_TEST(NoGps_Engine_ManualModeWhenGpsIsLostWhileNavigating)
{
  Env env(false /* inertial */);
  env.m_map.m_navigating = true;
  env.DriveGps(10, 0, 10);
  TEST_EQUAL(env.Last().m_provider, Provider::Gps, ());
  TEST(!env.m_engine.IsManualMode(), ());

  // GPS is jammed.
  env.m_clock.Advance(Engine::kGpsLostMs + 1500);
  TEST(env.m_engine.IsManualMode(), ());
  TEST(env.m_delegate.HasEvent(Event::GpsLost), ());
  // The car is where it was shown, on the road.
  TEST_EQUAL(env.Last().m_provider, Provider::Manual, ());
  TestAlmostEqualAbs(East(env.Last().m_position), 90, 1);

  // GPS works again for long enough near the car.
  for (int i = 0; i < 16; ++i)
  {
    env.m_clock.Advance(1000);
    env.m_engine.OnFix(GpsAt(env.m_clock, 90, 0, {}, 0.0));
  }
  TEST(!env.m_engine.IsManualMode(), ());
  TEST(env.m_delegate.HasEvent(Event::GpsBack), ());
  TEST_EQUAL(env.Last().m_provider, Provider::Gps, ());
}

UNIT_TEST(NoGps_Engine_ManualModeWhenGpsIsSpoofed)
{
  Env env(false /* inertial */);
  env.m_map.m_navigating = true;
  Fix network = GpsAt(env.m_clock, 0, 0, {}, {}, 100);
  network.m_provider = Provider::Network;
  env.m_engine.OnFix(network);
  env.DriveGps(3, 0, 10);
  TEST(!env.m_engine.IsManualMode(), ());

  // GPS is on another continent.
  env.m_clock.Advance(1000);
  Fix spoofed = GpsAt(env.m_clock, 0, 0, 90.0, 10.0);
  spoofed.m_position = {-12.0453, -77.0569};
  env.m_engine.OnFix(spoofed);
  TEST(env.m_engine.GetStatus().m_gpsSpoofed, ());
  TEST(env.m_delegate.HasEvent(Event::GpsSpoofed), ());
  TEST(env.m_engine.IsManualMode(), ());
  TEST_EQUAL(env.Last().m_provider, Provider::Manual, ());
}

UNIT_TEST(NoGps_Engine_ManualModeWhenGpsLeavesRoads)
{
  Env env(false /* inertial */);
  env.m_map.m_navigating = true;
  env.DriveGps(3, 0, 10);
  // A single position between the streets is noise.
  for (int i = 0; i + 1 < Engine::kOffRoadGpsCount; ++i)
  {
    env.m_clock.Advance(1000);
    env.m_engine.OnFix(GpsAt(env.m_clock, 50, 50, 90.0, 10.0));
  }
  TEST(!env.m_engine.IsManualMode(), ());
  env.m_clock.Advance(1000);
  env.m_engine.OnFix(GpsAt(env.m_clock, 50, 50, 90.0, 10.0));
  TEST(env.m_engine.IsManualMode(), ());
  // The car stays on the road where GPS was trusted the last time.
  TestAlmostEqualAbs(North(env.Last().m_position), 0, 0.5);
}

UNIT_TEST(NoGps_Engine_DisabledGpsIsIgnored)
{
  Env env(false /* inertial */);
  env.m_engine.SetGpsDisabled(true);
  env.DriveGps(3, 0, 10);
  TEST(env.m_delegate.m_positions.empty(), ());
  TEST(!env.m_engine.GetStatus().m_workingGpsAccuracyM, ());
}

UNIT_TEST(NoGps_Engine_InertialContinuesWhenGpsIsLost)
{
  Env env(true /* inertial */);
  // GPS gives the position and the heading while the box tells the speed.
  for (int i = 0; i < 5; ++i)
  {
    env.Drive(1000, 36, 0);
    env.m_engine.OnFix(GpsAt(env.m_clock, 10 * i, 0, 90.0, 10.0));
  }
  TEST_EQUAL(env.Last().m_provider, Provider::Gps, ());
  // GPS is lost: the manual mode is turned on by itself and the car is followed by the inertial navigation.
  env.Drive(Engine::kGpsLostMs + 2000, 36, 0);
  TEST(env.m_engine.IsManualMode(), ());
  TEST_EQUAL(env.Last().m_provider, Provider::Inertial, ());
  TEST_EQUAL(env.m_engine.GetStatus().m_source, PositionSource::Inertial, ());
  TEST_GREATER(East(env.Last().m_position), 90, ());
  TestAlmostEqualAbs(North(env.Last().m_position), 0, 1);
}
// The car speeds up to 54 km/h in 10 s, drives 5 s, brakes in 10 s and stands 5 s, again and again, going east
// along the street. The box reports the car speed 1.2 s late. Returns how far the calculated position was from the
// car at most 1.5-3 s after the car started to speed up or to brake, when the box has been learned.
double DriveWithLateSpeed(bool accelerometer)
{
  double constexpr kLagSec = 1.2;
  double constexpr kCycleSec = 30;
  auto const acceleration = [](double timeSec)
  {
    double const t = std::fmod(timeSec, kCycleSec);
    return t < 10 ? 1.5 : t >= 15 && t < 25 ? -1.5 : 0.0;
  };

  Env env(true /* inertial */);
  MarkCar(env, -450);
  int64_t seq = 1000;
  int64_t boxTimeMs = 200'000;
  double speed = 0;
  double east = -450;
  // The speeds of the car of the last seconds.
  std::deque<double> speeds(static_cast<size_t>(kLagSec / 0.02), 0.0);
  double maxError = 0;
  for (int step = 0; step < 4 * kCycleSec * 50; ++step)
  {
    double const t = step * 0.02;
    double const a = acceleration(t);
    east += speed * 0.02 + a * 0.02 * 0.02 / 2;
    speed = std::max(0.0, speed + a * 0.02);
    speeds.push_back(speed);
    int const reportedKmh = static_cast<int>(std::lround(speeds.front() * 3.6));
    speeds.pop_front();

    env.m_clock.Advance(20);
    boxTimeMs += 20;
    char body[160];
    if (accelerometer)
    {
      // The box looks forward with its first horizontal axis, a little of the gravity is left in the others.
      std::snprintf(body, sizeof(body), "NGD,1,%lld,%lld,0,0,0,%d,0,%X,%d,50,-60,100", static_cast<long long>(++seq),
                    static_cast<long long>(boxTimeMs), reportedKmh, kCalibratedFlags, static_cast<int>(a * 1000));
    }
    else
    {
      std::snprintf(body, sizeof(body), "NGD,1,%lld,%lld,0,0,0,%d,0,%X", static_cast<long long>(++seq),
                    static_cast<long long>(boxTimeMs), reportedKmh, kCalibratedFlags);
    }
    size_t const positions = env.m_delegate.m_positions.size();
    env.m_engine.OnEsp32Datagram(BoxLine(body));
    // The third and the fourth time the car speeds up and brakes.
    double const inCycle = std::fmod(t, kCycleSec);
    bool const changing = (inCycle >= 1.5 && inCycle < 3) || (inCycle >= 16.5 && inCycle < 18);
    if (t >= 2 * kCycleSec && changing && env.m_delegate.m_positions.size() > positions)
      maxError = std::max(maxError, std::fabs(East(env.Last().m_position) - east));
  }
  TEST_EQUAL(env.Last().m_provider, Provider::Inertial, ());
  TEST_EQUAL(env.m_engine.GetStatus().m_source, PositionSource::Inertial, ());
  return maxError;
}

UNIT_TEST(NoGps_Engine_BoxAccelerometerTellsSpeedBeforeCar)
{
  // The car reports the new speed late. The distance it has driven meanwhile is added when it does, so the
  // position is right in the end, but it is behind while the car speeds up and ahead while it brakes.
  double const bySpeed = DriveWithLateSpeed(false /* accelerometer */);
  // The accelerometer of the box feels the change at once.
  double const byAccel = DriveWithLateSpeed(true /* accelerometer */);
  LOG(LWARNING, ("The largest error by the car speed", bySpeed, "m, with the accelerometer", byAccel, "m"));
  TEST_GREATER(bySpeed, 0.8, ());
  TEST_LESS(bySpeed, 3, ());
  TEST_LESS(byAccel, bySpeed / 2, ());
}
}  // namespace nogps_engine_tests
