#ifndef RL_SAR_LOGGER_EVENT_LOGGER_HPP
#define RL_SAR_LOGGER_EVENT_LOGGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <fstream>

#include "rl_sar/logger/event_types.hpp"

namespace rl_sar::logger
{

/**
 * @brief Robot operational modes as defined in the frozen contract
 *
 * These correspond to the supervisor mode machine states defined in
 * doc/第1轮冻结契约.md section 5.1
 */
enum class ModeType : uint8_t
{
    BOOT = 0,
    PROBE = 1,
    PASSIVE = 2,
    READY = 3,
    RL_DOG_ONLY_ACTIVE = 4,
    MANUAL_ARM = 5,
    DEGRADED_ARM = 6,
    DEGRADED_BODY = 7,
    SOFT_STOP = 8,
    FAULT_LATCHED = 9
};

/**
 * @brief Fault types that can be logged
 *
 * These represent the key fault conditions defined in the frozen contract
 */
enum class FaultType : uint8_t
{
    BODY_STATE_STALE = 0,
    ARM_STATE_STALE = 1,
    POLICY_STALE = 2,
    ARM_TRACKING_ERROR = 3,
    SAFETY_LIMIT_EXCEEDED = 4,
    DDS_FAILURE = 5,
    CAN_FAILURE = 6,
    UNKNOWN = 7
};

/**
 * @brief Single event log entry containing all context
 *
 * This structure captures the complete state around a mode transition
 * or fault event for post-mortem analysis.
 */
struct EventLogEntry
{
    // Timestamp and sequencing
    uint64_t timestamp_ns;        ///< Monotonic timestamp (nanoseconds)
    uint64_t seq;                 ///< Global sequence number

    // Mode transition context
    ModeType from_mode;           ///< Previous mode
    ModeType to_mode;             ///< New mode after transition
    std::string reason;           ///< Human-readable reason for transition

    // Fault context
    bool is_fault;                ///< True if this entry represents a fault
    FaultType fault_type;         ///< Type of fault (if is_fault == true)
    std::string fault_details;    ///< Additional fault details

    // Context data captured at event time
    double body_state_age_us;     ///< Body state freshness (microseconds)
    double arm_state_age_us;      ///< Arm state freshness (microseconds)
    double arm_tracking_error;    ///< Current arm tracking error norm
    std::string current_policy_id;///< Active policy identifier

    // Additional metadata
    std::string source;           ///< Component that generated the event
    uint64_t detail_value;        ///< Optional numeric detail value
};

/**
 * @brief Configuration for the event logger
 */
struct EventLoggerConfig
{
    bool enable_file_logging = true;
    bool enable_console_logging = true;
    bool enable_json_logging = true;
    std::string log_file_path = "/tmp/go2_x5_events.log";
    std::string json_log_path = "/tmp/go2_x5_events.jsonl";
    size_t max_memory_entries = 1000;
    bool flush_on_write = true;
    bool include_context_in_faults = true;
};

/**
 * @brief Thread-safe event logger for mode transitions and faults
 *
 * The EventLogger provides structured logging for all mode transitions
 * and fault conditions with full context capture. It maintains an in-memory
 * buffer for retrieval and supports both human-readable and JSON output.
 *
 * Key features:
 * - Thread-safe event logging
 * - Configurable file and console output
 * - JSON Lines format for easy parsing
 * - In-memory circular buffer for event retrieval
 * - Automatic sequence numbering
 *
 * Example usage:
 * @code
 * EventLogger logger;
 * EventLoggerConfig config;
 * config.enable_console_logging = true;
 * logger.Initialize(config);
 *
 * logger.LogModeTransition(ModeType::READY, ModeType::RL_DOG_ONLY_ACTIVE,
 *                          "Operator enabled");
 *
 * logger.LogFault(FaultType::BODY_STATE_STALE, "Body state age: 55ms");
 * @endcode
 */
class EventLogger
{
public:
    EventLogger();
    ~EventLogger();

    // Non-copyable, non-movable
    EventLogger(const EventLogger&) = delete;
    EventLogger& operator=(const EventLogger&) = delete;
    EventLogger(EventLogger&&) = delete;
    EventLogger& operator=(EventLogger&&) = delete;

    /**
     * @brief Initialize the event logger with configuration
     * @param config Configuration settings
     * @return true if initialization successful
     */
    bool Initialize(const EventLoggerConfig& config);

    /**
     * @brief Shutdown the logger and flush all buffers
     */
    void Shutdown();

    /**
     * @brief Log a mode transition with full context
     * @param from Source mode
     * @param to Destination mode
     * @param reason Human-readable reason for the transition
     */
    void LogModeTransition(ModeType from, ModeType to, const std::string& reason);

    /**
     * @brief Log a mode transition with additional context
     * @param from Source mode
     * @param to Destination mode
     * @param reason Human-readable reason
     * @param body_state_age_us Body state freshness
     * @param arm_state_age_us Arm state freshness
     * @param arm_tracking_error Current tracking error
     * @param policy_id Active policy ID
     */
    void LogModeTransition(
        ModeType from,
        ModeType to,
        const std::string& reason,
        double body_state_age_us,
        double arm_state_age_us,
        double arm_tracking_error,
        const std::string& policy_id);

    /**
     * @brief Log a fault condition
     * @param type Type of fault
     * @param details Human-readable fault details
     */
    void LogFault(FaultType type, const std::string& details);

    /**
     * @brief Log a fault with full context
     * @param type Type of fault
     * @param details Fault details
     * @param body_state_age_us Body state freshness
     * @param arm_state_age_us Arm state freshness
     * @param arm_tracking_error Current tracking error
     * @param policy_id Active policy ID
     * @param detail_value Optional numeric detail value
     */
    void LogFault(
        FaultType type,
        const std::string& details,
        double body_state_age_us,
        double arm_state_age_us,
        double arm_tracking_error,
        const std::string& policy_id,
        uint64_t detail_value = 0);

    /**
     * @brief Log a generic event with full entry specification
     * @param event Complete event entry
     */
    void LogEvent(const EventLogEntry& event);

    /**
     * @brief Log a structured runtime event using the unified stage-2 model
     * @param event Structured event
     */
    void Log(const Event& event);

    /**
     * @brief Get all events from in-memory buffer
     * @return Copy of all stored events
     */
    std::vector<EventLogEntry> GetEvents() const;

    /**
     * @brief Get only fault events
     * @return Vector of fault events
     */
    std::vector<EventLogEntry> GetFaultEvents() const;

    /**
     * @brief Get events filtered by mode
     * @param mode Mode to filter by (checks both from_mode and to_mode)
     * @return Events involving the specified mode
     */
    std::vector<EventLogEntry> GetEventsByMode(ModeType mode) const;

    /**
     * @brief Get events filtered by fault type
     * @param type Fault type to filter
     * @return Events of the specified fault type
     */
    std::vector<EventLogEntry> GetEventsByFaultType(FaultType type) const;

    /**
     * @brief Get events since a specific timestamp
     * @param timestamp_ns Starting timestamp (nanoseconds)
     * @return Events with timestamp >= timestamp_ns
     */
    std::vector<EventLogEntry> GetEventsSince(uint64_t timestamp_ns) const;

    std::vector<Event> GetRecentEvents(size_t count = 10) const;
    std::vector<Event> GetEventsByCategory(EventCategory category) const;
    std::vector<Event> GetEventsBySeverity(EventSeverity min_severity) const;
    std::string GenerateSummary() const;

    /**
     * @brief Clear the in-memory event buffer
     */
    void Clear();

    /**
     * @brief Get current event count
     * @return Number of events stored
     */
    size_t GetEventCount() const;

    /**
     * @brief Force flush buffers to disk
     */
    void Flush();

    /**
     * @brief Check if logger is initialized
     * @return true if Initialize() was called successfully
     */
    bool IsInitialized() const { return initialized_; }

    /**
     * @brief Get the current sequence number
     * @return Next sequence number to be assigned
     */
    uint64_t GetNextSequenceNumber() const { return next_seq_; }

    /**
     * @brief Convert ModeType to string
     * @param mode Mode to convert
     * @return String representation
     */
    static const char* ModeToString(ModeType mode);

    /**
     * @brief Convert FaultType to string
     * @param type Fault type to convert
     * @return String representation
     */
    static const char* FaultToString(FaultType type);

private:
    // Internal implementation
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Configuration and state
    EventLoggerConfig config_;
    bool initialized_;
    uint64_t next_seq_;

    // Thread safety
    mutable std::mutex mutex_;

    // Internal methods
    void WriteToLog(const EventLogEntry& event);
    void WriteToJson(const EventLogEntry& event);
    void WriteToConsole(const EventLogEntry& event);
    void FlushUnlocked();
    void AddToBuffer(const EventLogEntry& event);
    std::string FormatEvent(const EventLogEntry& event) const;
    std::string FormatJson(const EventLogEntry& event) const;
    std::string EscapeJsonString(const std::string& str) const;
    Event ToStructuredEvent(const EventLogEntry& event) const;
    EventLogEntry FromStructuredEvent(const Event& event) const;
    uint64_t GetTimestampNs() const;
};

} // namespace rl_sar::logger

#endif // RL_SAR_LOGGER_EVENT_LOGGER_HPP
