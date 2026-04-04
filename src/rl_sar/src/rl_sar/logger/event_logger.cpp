#include "rl_sar/logger/event_logger.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include <cmath>

namespace rl_sar::logger
{

// PIMPL implementation details
struct EventLogger::Impl
{
    std::ofstream log_file;
    std::ofstream json_file;
    std::vector<EventLogEntry> event_buffer;
};

// ============================================================================
// Static string conversion functions
// ============================================================================

const char* EventLogger::ModeToString(ModeType mode)
{
    switch (mode)
    {
        case ModeType::BOOT:              return "BOOT";
        case ModeType::PROBE:             return "PROBE";
        case ModeType::PASSIVE:           return "PASSIVE";
        case ModeType::READY:             return "READY";
        case ModeType::RL_DOG_ONLY_ACTIVE: return "RL_DOG_ONLY_ACTIVE";
        case ModeType::MANUAL_ARM:        return "MANUAL_ARM";
        case ModeType::DEGRADED_ARM:      return "DEGRADED_ARM";
        case ModeType::DEGRADED_BODY:     return "DEGRADED_BODY";
        case ModeType::SOFT_STOP:         return "SOFT_STOP";
        case ModeType::FAULT_LATCHED:     return "FAULT_LATCHED";
    }
    return "UNKNOWN";
}

const char* EventLogger::FaultToString(FaultType type)
{
    switch (type)
    {
        case FaultType::BODY_STATE_STALE:      return "BODY_STATE_STALE";
        case FaultType::ARM_STATE_STALE:       return "ARM_STATE_STALE";
        case FaultType::POLICY_STALE:          return "POLICY_STALE";
        case FaultType::ARM_TRACKING_ERROR:    return "ARM_TRACKING_ERROR";
        case FaultType::SAFETY_LIMIT_EXCEEDED: return "SAFETY_LIMIT_EXCEEDED";
        case FaultType::DDS_FAILURE:           return "DDS_FAILURE";
        case FaultType::CAN_FAILURE:           return "CAN_FAILURE";
        case FaultType::UNKNOWN:               return "UNKNOWN";
    }
    return "UNKNOWN";
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

EventLogger::EventLogger()
    : impl_(std::make_unique<Impl>())
    , initialized_(false)
    , next_seq_(1)
{
}

EventLogger::~EventLogger()
{
    Shutdown();
}

// ============================================================================
// Initialization
// ============================================================================

bool EventLogger::Initialize(const EventLoggerConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Already initialized
    if (initialized_)
    {
        return true;
    }

    config_ = config;
    impl_->event_buffer.reserve(config_.max_memory_entries);

    // Open text log file
    if (config_.enable_file_logging)
    {
        impl_->log_file.open(config_.log_file_path,
                             std::ios::out | std::ios::app);
        if (!impl_->log_file.is_open())
        {
            // Fall back to console logging if file fails
            config_.enable_file_logging = false;
        }
    }

    // Open JSON log file
    if (config_.enable_json_logging)
    {
        impl_->json_file.open(config_.json_log_path,
                              std::ios::out | std::ios::app);
        if (!impl_->json_file.is_open())
        {
            config_.enable_json_logging = false;
        }
    }

    initialized_ = true;

    // Log initialization event
    EventLogEntry init_entry;
    init_entry.timestamp_ns = GetTimestampNs();
    init_entry.seq = next_seq_++;
    init_entry.from_mode = ModeType::BOOT;
    init_entry.to_mode = ModeType::BOOT;
    init_entry.reason = "EventLogger initialized";
    init_entry.is_fault = false;
    init_entry.fault_type = FaultType::UNKNOWN;
    init_entry.body_state_age_us = 0.0;
    init_entry.arm_state_age_us = 0.0;
    init_entry.arm_tracking_error = 0.0;
    init_entry.current_policy_id = "";
    init_entry.source = "EventLogger";
    init_entry.detail_value = 0;

    AddToBuffer(init_entry);
    WriteToLog(init_entry);
    WriteToJson(init_entry);
    WriteToConsole(init_entry);

    return true;
}

void EventLogger::Shutdown()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_)
    {
        return;
    }

    // Log shutdown event
    EventLogEntry shutdown_entry;
    shutdown_entry.timestamp_ns = GetTimestampNs();
    shutdown_entry.seq = next_seq_++;
    shutdown_entry.from_mode = ModeType::BOOT;  // Unknown current mode
    shutdown_entry.to_mode = ModeType::BOOT;
    shutdown_entry.reason = "EventLogger shutdown";
    shutdown_entry.is_fault = false;
    shutdown_entry.fault_type = FaultType::UNKNOWN;
    shutdown_entry.body_state_age_us = 0.0;
    shutdown_entry.arm_state_age_us = 0.0;
    shutdown_entry.arm_tracking_error = 0.0;
    shutdown_entry.current_policy_id = "";
    shutdown_entry.source = "EventLogger";
    shutdown_entry.detail_value = 0;

    AddToBuffer(shutdown_entry);
    WriteToLog(shutdown_entry);
    WriteToJson(shutdown_entry);
    WriteToConsole(shutdown_entry);

    // Close files
    if (impl_->log_file.is_open())
    {
        impl_->log_file.close();
    }
    if (impl_->json_file.is_open())
    {
        impl_->json_file.close();
    }

    initialized_ = false;
}

// ============================================================================
// Mode transition logging
// ============================================================================

void EventLogger::LogModeTransition(ModeType from, ModeType to,
                                    const std::string& reason)
{
    LogModeTransition(from, to, reason, 0.0, 0.0, 0.0, "");
}

void EventLogger::LogModeTransition(
    ModeType from,
    ModeType to,
    const std::string& reason,
    double body_state_age_us,
    double arm_state_age_us,
    double arm_tracking_error,
    const std::string& policy_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EventLogEntry entry;
    entry.timestamp_ns = GetTimestampNs();
    entry.seq = next_seq_++;
    entry.from_mode = from;
    entry.to_mode = to;
    entry.reason = reason;
    entry.is_fault = false;
    entry.fault_type = FaultType::UNKNOWN;
    entry.fault_details = "";
    entry.body_state_age_us = body_state_age_us;
    entry.arm_state_age_us = arm_state_age_us;
    entry.arm_tracking_error = arm_tracking_error;
    entry.current_policy_id = policy_id;
    entry.source = "supervisor";
    entry.detail_value = 0;

    AddToBuffer(entry);
    WriteToLog(entry);
    WriteToJson(entry);
    WriteToConsole(entry);

    if (config_.flush_on_write)
    {
        Flush();
    }
}

// ============================================================================
// Fault logging
// ============================================================================

void EventLogger::LogFault(FaultType type, const std::string& details)
{
    LogFault(type, details, 0.0, 0.0, 0.0, "", 0);
}

void EventLogger::LogFault(
    FaultType type,
    const std::string& details,
    double body_state_age_us,
    double arm_state_age_us,
    double arm_tracking_error,
    const std::string& policy_id,
    uint64_t detail_value)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EventLogEntry entry;
    entry.timestamp_ns = GetTimestampNs();
    entry.seq = next_seq_++;
    entry.from_mode = ModeType::BOOT;  // Will be filled by caller if needed
    entry.to_mode = ModeType::FAULT_LATCHED;
    entry.reason = "Fault detected";
    entry.is_fault = true;
    entry.fault_type = type;
    entry.fault_details = details;
    entry.body_state_age_us = body_state_age_us;
    entry.arm_state_age_us = arm_state_age_us;
    entry.arm_tracking_error = arm_tracking_error;
    entry.current_policy_id = policy_id;
    entry.source = "supervisor";
    entry.detail_value = detail_value;

    AddToBuffer(entry);
    WriteToLog(entry);
    WriteToJson(entry);
    WriteToConsole(entry);

    if (config_.flush_on_write)
    {
        Flush();
    }
}

// ============================================================================
// Generic event logging
// ============================================================================

void EventLogger::LogEvent(const EventLogEntry& event)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EventLogEntry entry = event;  // Copy
    if (entry.seq == 0)
    {
        entry.seq = next_seq_++;
    }
    else
    {
        next_seq_ = std::max(next_seq_, entry.seq + 1);
    }

    if (entry.timestamp_ns == 0)
    {
        entry.timestamp_ns = GetTimestampNs();
    }

    AddToBuffer(entry);
    WriteToLog(entry);
    WriteToJson(entry);
    WriteToConsole(entry);

    if (config_.flush_on_write)
    {
        Flush();
    }
}

// ============================================================================
// Event retrieval
// ============================================================================

std::vector<EventLogEntry> EventLogger::GetEvents() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->event_buffer;
}

std::vector<EventLogEntry> EventLogger::GetFaultEvents() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<EventLogEntry> faults;
    faults.reserve(impl_->event_buffer.size());

    for (const auto& entry : impl_->event_buffer)
    {
        if (entry.is_fault)
        {
            faults.push_back(entry);
        }
    }

    return faults;
}

std::vector<EventLogEntry> EventLogger::GetEventsByMode(ModeType mode) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<EventLogEntry> result;
    result.reserve(impl_->event_buffer.size());

    for (const auto& entry : impl_->event_buffer)
    {
        if (entry.from_mode == mode || entry.to_mode == mode)
        {
            result.push_back(entry);
        }
    }

    return result;
}

std::vector<EventLogEntry> EventLogger::GetEventsByFaultType(FaultType type) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<EventLogEntry> result;
    result.reserve(impl_->event_buffer.size());

    for (const auto& entry : impl_->event_buffer)
    {
        if (entry.is_fault && entry.fault_type == type)
        {
            result.push_back(entry);
        }
    }

    return result;
}

std::vector<EventLogEntry> EventLogger::GetEventsSince(uint64_t timestamp_ns) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<EventLogEntry> result;
    result.reserve(impl_->event_buffer.size());

    for (const auto& entry : impl_->event_buffer)
    {
        if (entry.timestamp_ns >= timestamp_ns)
        {
            result.push_back(entry);
        }
    }

    return result;
}

void EventLogger::Clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    impl_->event_buffer.clear();
    // Don't reset next_seq_ to maintain global ordering
}

size_t EventLogger::GetEventCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->event_buffer.size();
}

void EventLogger::Flush()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (impl_->log_file.is_open())
    {
        impl_->log_file.flush();
    }
    if (impl_->json_file.is_open())
    {
        impl_->json_file.flush();
    }
}

// ============================================================================
// Internal helpers
// ============================================================================

void EventLogger::AddToBuffer(const EventLogEntry& event)
{
    // Manage circular buffer
    if (impl_->event_buffer.size() >= config_.max_memory_entries)
    {
        // Remove oldest entry (fifo)
        impl_->event_buffer.erase(impl_->event_buffer.begin());
    }
    impl_->event_buffer.push_back(event);
}

void EventLogger::WriteToLog(const EventLogEntry& event)
{
    if (!config_.enable_file_logging || !impl_->log_file.is_open())
    {
        return;
    }

    impl_->log_file << FormatEvent(event) << std::endl;
}

void EventLogger::WriteToJson(const EventLogEntry& event)
{
    if (!config_.enable_json_logging || !impl_->json_file.is_open())
    {
        return;
    }

    impl_->json_file << FormatJson(event) << std::endl;
}

void EventLogger::WriteToConsole(const EventLogEntry& event)
{
    if (!config_.enable_console_logging)
    {
        return;
    }

    std::cout << FormatEvent(event) << std::endl;
}

std::string EventLogger::FormatEvent(const EventLogEntry& event) const
{
    std::ostringstream oss;

    // Format timestamp as seconds since epoch with decimal nanoseconds
    double timestamp_sec = event.timestamp_ns / 1e9;

    oss << std::fixed << std::setprecision(9)
        << "[" << timestamp_sec << "] "
        << "[seq:" << event.seq << "] ";

    if (event.is_fault)
    {
        oss << "[FAULT] "
            << FaultToString(event.fault_type) << ": "
            << event.fault_details;
    }
    else
    {
        oss << "[TRANSITION] "
            << ModeToString(event.from_mode) << " -> "
            << ModeToString(event.to_mode)
            << " | Reason: " << event.reason;
    }

    // Add context if available
    if (event.body_state_age_us > 0 || event.arm_state_age_us > 0 ||
        event.arm_tracking_error > 0 || !event.current_policy_id.empty())
    {
        oss << " | Context: ";
        bool first = true;

        if (event.body_state_age_us > 0)
        {
            oss << (first ? "" : ", ") << "body_age=" << event.body_state_age_us << "us";
            first = false;
        }
        if (event.arm_state_age_us > 0)
        {
            oss << (first ? "" : ", ") << "arm_age=" << event.arm_state_age_us << "us";
            first = false;
        }
        if (event.arm_tracking_error > 0)
        {
            oss << (first ? "" : ", ") << "tracking_err=" << event.arm_tracking_error;
            first = false;
        }
        if (!event.current_policy_id.empty())
        {
            oss << (first ? "" : ", ") << "policy=" << event.current_policy_id;
            first = false;
        }
    }

    if (event.detail_value != 0)
    {
        oss << " | detail=" << event.detail_value;
    }

    if (!event.source.empty() && event.source != "supervisor")
    {
        oss << " | src=" << event.source;
    }

    return oss.str();
}

std::string EventLogger::FormatJson(const EventLogEntry& event) const
{
    std::ostringstream oss;

    oss << "{"
        << "\"ts_ns\":" << event.timestamp_ns << ","
        << "\"seq\":" << event.seq << ","
        << "\"is_fault\":" << (event.is_fault ? "true" : "false") << ",";

    if (event.is_fault)
    {
        oss << "\"fault_type\":\"" << FaultToString(event.fault_type) << "\","
            << "\"fault_details\":\"" << EscapeJsonString(event.fault_details) << "\",";
    }
    else
    {
        oss << "\"from_mode\":\"" << ModeToString(event.from_mode) << "\","
            << "\"to_mode\":\"" << ModeToString(event.to_mode) << "\","
            << "\"reason\":\"" << EscapeJsonString(event.reason) << "\",";
    }

    oss << "\"body_state_age_us\":" << event.body_state_age_us << ","
        << "\"arm_state_age_us\":" << event.arm_state_age_us << ","
        << "\"arm_tracking_error\":" << event.arm_tracking_error << ","
        << "\"policy_id\":\"" << EscapeJsonString(event.current_policy_id) << "\","
        << "\"source\":\"" << event.source << "\","
        << "\"detail_value\":" << event.detail_value
        << "}";

    return oss.str();
}

std::string EventLogger::EscapeJsonString(const std::string& str) const
{
    std::string result;
    result.reserve(str.size() * 2);

    for (char c : str)
    {
        switch (c)
        {
            case '"':  result.append("\\\""); break;
            case '\\': result.append("\\\\"); break;
            case '\b': result.append("\\b"); break;
            case '\f': result.append("\\f"); break;
            case '\n': result.append("\\n"); break;
            case '\r': result.append("\\r"); break;
            case '\t': result.append("\\t"); break;
            default:
                if (c < 32)
                {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04x", static_cast<int>(c));
                    result.append(buf);
                }
                else
                {
                    result.push_back(c);
                }
                break;
        }
    }

    return result;
}

uint64_t EventLogger::GetTimestampNs() const
{
    // Get steady clock time
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

} // namespace rl_sar::logger
