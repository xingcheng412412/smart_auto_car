#ifndef CAN_CLIENT__COMMAND_WATCHDOG_HPP_
#define CAN_CLIENT__COMMAND_WATCHDOG_HPP_
#include <chrono>
#include <cmath>
#include <stdexcept>

// Wall-clock watchdog: pausing or resetting a ROS clock cannot prolong a command.
class CommandWatchdog
{
public:
    explicit CommandWatchdog(double timeout_seconds) : timeout_(timeout_seconds)
    {
        if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0) {
            throw std::invalid_argument("command_timeout_sec must be finite and positive");
        }
    }
    void received(std::chrono::steady_clock::time_point stamp)
    {
        last_ = stamp;
        valid_ = true;
    }
    void invalidate() {valid_ = false;}
    bool fresh(std::chrono::steady_clock::time_point now) const
    {
        return valid_ && now >= last_ && now - last_ <= timeout_;
    }
private:
    std::chrono::duration<double> timeout_;
    std::chrono::steady_clock::time_point last_{};
    bool valid_{false};
};
#endif
