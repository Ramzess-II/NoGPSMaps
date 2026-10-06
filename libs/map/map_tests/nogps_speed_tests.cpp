#include "testing/testing.hpp"

#include "map/nogps/speed_lag.hpp"
#include "map/nogps/speed_table.hpp"

#include <cmath>

namespace nogps_speed_tests
{
using namespace nogps;

// The car accelerates to 15 m/s in 10 s, drives 10 s, brakes in 10 s and stands 10 s.
double TrueSpeed(double timeSec)
{
  double const t = std::fmod(std::fmod(timeSec, 40) + 40, 40);
  if (t < 10)
    return 1.5 * t;
  if (t < 20)
    return 15;
  if (t < 30)
    return 1.5 * (30 - t);
  return 0;
}

// Drives for the time: the car reports its speed 5 times a second rounded to 1 km/h and late by the lag,
// GPS measures the real speed every second.
void DriveWithLag(SpeedLag & lag, double lagSec, int seconds)
{
  for (int ms = 0; ms < seconds * 1000; ms += 200)
  {
    lag.OnCarSpeed(ms, std::floor(TrueSpeed(ms / 1000.0 - lagSec) * 3.6) / 3.6);
    if (ms % 1000 == 0)
      lag.OnGpsSpeed(ms, TrueSpeed(ms / 1000.0));
  }
}

UNIT_TEST(NoGps_SpeedLag_UsualLagUntilMeasured)
{
  SpeedLag lag;
  TEST(!lag.IsMeasured(), ());
  TEST_EQUAL(lag.Get(), SpeedLag::kDefaultLagSec, ());
  // A steady speed tells nothing.
  for (int ms = 0; ms < 600'000; ms += 200)
  {
    lag.OnCarSpeed(ms, 15);
    if (ms % 1000 == 0)
      lag.OnGpsSpeed(ms, 15);
  }
  TEST(!lag.IsMeasured(), ());
}

UNIT_TEST(NoGps_SpeedLag_MeasuresLag)
{
  for (double const lagSec : {0.4, 1.0, 1.4, 2.2})
  {
    SpeedLag lag;
    DriveWithLag(lag, lagSec, 400);
    TEST(lag.IsMeasured(), (lagSec));
    TEST_ALMOST_EQUAL_ABS(lag.Get(), lagSec, 0.2, ());
  }
}

UNIT_TEST(NoGps_SpeedLag_FollowsChangedLag)
{
  SpeedLag lag;
  DriveWithLag(lag, 0.5, 400);
  // Another adapter.
  DriveWithLag(lag, 1.5, 6000);
  TEST_ALMOST_EQUAL_ABS(lag.Get(), 1.5, 0.2, ());
}

UNIT_TEST(NoGps_SpeedLag_IsKeptBetweenTrips)
{
  SpeedLag lag;
  DriveWithLag(lag, 1.6, 400);
  SpeedLag restored;
  restored.Deserialize(lag.Serialize());
  TEST(restored.IsMeasured(), ());
  TEST_ALMOST_EQUAL_ABS(restored.Get(), lag.Get(), 0.01, ());
  restored.Deserialize("broken");
  TEST(!restored.IsMeasured(), ());
  restored.Deserialize(lag.Serialize());
  restored.Clear();
  TEST_EQUAL(restored.Get(), SpeedLag::kDefaultLagSec, ());
}

// Drives at a steady speed for |seconds|, GPS tells the real speed.
void DriveSteady(SpeedTable & table, double carKmh, double realKmh, int seconds)
{
  for (int i = 0; i < seconds; ++i)
    table.OnSample(carKmh, realKmh / 3.6, 1, 0);
}

UNIT_TEST(NoGps_SpeedTable_UnknownWithoutSamples)
{
  TEST(!SpeedTable().GetScale(50), ());
}

UNIT_TEST(NoGps_SpeedTable_MeasuresEveryRange)
{
  SpeedTable table;
  // The speedometer is 5% optimistic at 50 km/h and 2% at 90 km/h.
  DriveSteady(table, 52.5, 50, 60);
  DriveSteady(table, 91.8, 90, 60);
  TEST_ALMOST_EQUAL_ABS(*table.GetScale(55), 50 / 52.5, 1e-9, ());
  TEST_ALMOST_EQUAL_ABS(*table.GetScale(95), 90 / 91.8, 1e-9, ());
  TEST_EQUAL(table.GetKnownRanges(), 2, ());
}

UNIT_TEST(NoGps_SpeedTable_InterpolatesUnknownRanges)
{
  SpeedTable table;
  DriveSteady(table, 30, 27, 60);
  DriveSteady(table, 70, 70, 60);
  // 50 km/h is in the middle between the measured 30 and 70 km/h ranges.
  TEST_ALMOST_EQUAL_ABS(*table.GetScale(50), (0.9 + 1.0) / 2, 1e-9, ());
  // Outside the measured ranges the closest one is used.
  TEST_ALMOST_EQUAL_ABS(*table.GetScale(10), 0.9, 1e-9, ());
  TEST_ALMOST_EQUAL_ABS(*table.GetScale(120), 1.0, 1e-9, ());
}

UNIT_TEST(NoGps_SpeedTable_NeedsDistance)
{
  SpeedTable table;
  // 100 m only.
  DriveSteady(table, 36, 30, 10);
  TEST(!table.GetScale(36), ());
}

UNIT_TEST(NoGps_SpeedTable_IgnoresBadSamples)
{
  SpeedTable table;
  for (int i = 0; i < 100; ++i)
  {
    // Too slow, accelerating, after a pause.
    table.OnSample(3, 1, 1, 0);
    table.OnSample(50, 20, 1, 2);
    table.OnSample(50, 20, 5, 0);
  }
  TEST_EQUAL(table.GetKnownRanges(), 0, ());
}

UNIT_TEST(NoGps_SpeedTable_ForgetsOldSamples)
{
  SpeedTable table;
  DriveSteady(table, 60, 54, 600);
  // Much longer with the new tyres: the old ratio fades away.
  DriveSteady(table, 60, 60, 3000);
  TEST_ALMOST_EQUAL_ABS(*table.GetScale(60), 1.0, 0.02, ());
}

UNIT_TEST(NoGps_SpeedTable_KeepsBetweenTrips)
{
  SpeedTable table;
  DriveSteady(table, 52.5, 50, 60);
  SpeedTable restored;
  restored.Deserialize(table.Serialize());
  TEST_ALMOST_EQUAL_ABS(*restored.GetScale(50), *table.GetScale(50), 1e-3, ());

  restored.Deserialize("broken");
  TEST_EQUAL(restored.GetKnownRanges(), 0, ());
}
}  // namespace nogps_speed_tests
