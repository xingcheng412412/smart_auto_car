#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include "command_watchdog.hpp"
using namespace std::chrono_literals;
TEST(CommandWatchdog, StartupAndTimeoutAreDisabled)
{
  CommandWatchdog guard(.25);
  const auto start=std::chrono::steady_clock::time_point(10s);
  EXPECT_FALSE(guard.fresh(start));
  guard.received(start);
  EXPECT_TRUE(guard.fresh(start+249ms));
  EXPECT_FALSE(guard.fresh(start+251ms));
}
TEST(CommandWatchdog, FreshCommandRecoversButBackwardTimeAndInvalidationStop)
{
  CommandWatchdog guard(.25);
  const auto start=std::chrono::steady_clock::time_point(10s);
  guard.received(start);
  EXPECT_FALSE(guard.fresh(start-1ms));
  guard.received(start+1s);
  EXPECT_TRUE(guard.fresh(start+1100ms));
  guard.invalidate();
  EXPECT_FALSE(guard.fresh(start+1100ms));
}
TEST(CommandWatchdog, RejectsInvalidTimeout)
{
  EXPECT_THROW(CommandWatchdog{0},std::invalid_argument);
  EXPECT_THROW(CommandWatchdog{-1},std::invalid_argument);
  EXPECT_THROW(CommandWatchdog{std::numeric_limits<double>::quiet_NaN()},std::invalid_argument);
}
