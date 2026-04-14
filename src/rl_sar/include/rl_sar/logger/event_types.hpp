#ifndef RL_SAR_LOGGER_EVENT_TYPES_HPP
#define RL_SAR_LOGGER_EVENT_TYPES_HPP

#include <cstdint>
#include <string>

namespace rl_sar::logger
{

enum class EventCategory
{
    ModeChange,
    Policy,
    Arm,
    Body,
    System,
};

enum class EventSeverity
{
    Info,
    Warning,
    Error,
    Critical,
};

struct Event
{
    uint64_t timestamp_ns = 0;
    EventCategory category = EventCategory::System;
    EventSeverity severity = EventSeverity::Info;
    std::string source;
    std::string message;
    std::string from_mode;
    std::string to_mode;
    std::string reason_code;
    bool policy_stale = false;
    uint64_t policy_age_ns = 0;
    bool arm_tracking_error = false;
    double arm_tracking_error_value = 0.0;
};

const char* ToString(EventCategory category);
const char* ToString(EventSeverity severity);

}  // namespace rl_sar::logger

#endif  // RL_SAR_LOGGER_EVENT_TYPES_HPP
