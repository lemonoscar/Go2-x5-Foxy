#ifndef RL_SAR_RUNTIME_SUPERVISOR_GO2_X5_SUPERVISOR_TYPES_HPP
#define RL_SAR_RUNTIME_SUPERVISOR_GO2_X5_SUPERVISOR_TYPES_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace Go2X5Supervisor
{

enum class Mode : uint8_t
{
    Boot = 0,
    Probe = 1,
    Passive = 2,
    Ready = 3,
    RlDogOnlyActive = 4,
    ManualArm = 5,
    DegradedArm = 6,
    DegradedBody = 7,
    SoftStop = 8,
    FaultLatched = 9,
};

enum class EventType : uint8_t
{
    BootComplete = 0,
    ProbePass = 1,
    ProbeFail = 2,
    OperatorEnable = 3,
    OperatorDisable = 4,
    PolicyHealthOk = 5,
    PolicyHealthBad = 6,
    BodyStateStale = 7,
    ArmStateStale = 8,
    ArmTrackingErrorHigh = 9,
    Estop = 10,
    SoftStopRequest = 11,
    FaultReset = 12,
};

enum class ReasonCode : uint32_t
{
    None = 0,
    ProbeFailed = 1001,
    ManifestMismatch = 1002,
    BodyStateStale = 1003,
    ArmStateStale = 1004,
    ArmTrackingErrorTooHigh = 1005,
    PolicyTimeout = 1006,
    DdsWriteFail = 1007,
    Estop = 1008,
    CommandExpired = 1009,
    IllegalJointMapping = 1010,
};

struct Config
{
    uint64_t probe_window_us = 2'000'000;
    uint64_t body_state_stale_us = 50'000;
    uint64_t arm_state_stale_us = 50'000;
    uint64_t policy_stale_us = 100'000;
    uint64_t arm_tracking_error_window_us = 200'000;
    uint64_t degraded_timeout_us = 1'500'000;
    uint64_t soft_stop_duration_us = 500'000;
    bool require_manifest_valid = true;
    bool fault_latched_requires_manual_reset = true;
};

struct WatchdogInput
{
    uint64_t now_monotonic_ns = 0;

    uint64_t body_state_age_us = 0;
    uint64_t arm_state_age_us = 0;
    uint64_t policy_age_us = 0;
    uint64_t arm_tracking_error_age_us = 0;

    bool manifest_valid = true;
    bool config_loaded = false;
    bool boot_complete = false;
    bool probe_pass = false;
    bool probe_fail = false;

    bool operator_enable = false;
    bool operator_disable = false;
    bool manual_arm_request = false;

    bool policy_health_ok = true;
    bool policy_health_bad = false;

    bool body_dds_write_ok = true;
    bool arm_backend_valid = true;
    bool arm_tracking_error_high = false;

    bool estop = false;
    bool soft_stop_request = false;
    bool fault_reset = false;
    bool allow_recover = false;

    bool has_body_state_seq = false;
    bool has_arm_state_seq = false;
    bool has_policy_seq = false;
    uint64_t body_state_seq = 0;
    uint64_t arm_state_seq = 0;
    uint64_t policy_seq = 0;
};

struct WatchdogStatus
{
    uint64_t now_monotonic_ns = 0;
    uint64_t heartbeat_age_us = 0;
    uint64_t body_state_age_us = 0;
    uint64_t arm_state_age_us = 0;
    uint64_t policy_age_us = 0;
    uint64_t arm_tracking_error_age_us = 0;

    bool manifest_valid = true;
    bool heartbeat_fresh = false;
    bool body_state_fresh = false;
    bool arm_state_fresh = false;
    bool policy_fresh = false;

    bool body_seq_gap = false;
    bool arm_seq_gap = false;
    bool policy_seq_gap = false;
    uint32_t seq_gap_count = 0;

    bool policy_health_ok = true;
    bool body_dds_write_ok = true;
    bool arm_backend_valid = true;
    bool arm_tracking_error_high = false;
    bool arm_tracking_error_window_exceeded = false;

    bool ready_gate_ok = false;
    bool active_gate_ok = false;
};

struct ModeEventRecord
{
    Mode from_mode = Mode::Boot;
    Mode to_mode = Mode::Boot;
    ReasonCode reason_code = ReasonCode::None;
    uint64_t trigger_seq = 0;
    uint64_t source_monotonic_ns = 0;
    uint64_t detail_value = 0;
    EventType event_type = EventType::BootComplete;
    std::string source_label = "supervisor";
};

struct TransitionResult
{
    bool mode_changed = false;
    Mode previous_mode = Mode::Boot;
    Mode current_mode = Mode::Boot;
    ReasonCode reason_code = ReasonCode::None;
    uint64_t trigger_seq = 0;
    uint64_t source_monotonic_ns = 0;
    uint64_t detail_value = 0;
    WatchdogStatus watchdog;
};

inline const char* ToString(Mode mode)
{
    switch (mode)
    {
    case Mode::Boot: return "BOOT";
    case Mode::Probe: return "PROBE";
    case Mode::Passive: return "PASSIVE";
    case Mode::Ready: return "READY";
    case Mode::RlDogOnlyActive: return "RL_DOG_ONLY_ACTIVE";
    case Mode::ManualArm: return "MANUAL_ARM";
    case Mode::DegradedArm: return "DEGRADED_ARM";
    case Mode::DegradedBody: return "DEGRADED_BODY";
    case Mode::SoftStop: return "SOFT_STOP";
    case Mode::FaultLatched: return "FAULT_LATCHED";
    }
    return "UNKNOWN";
}

inline const char* ToString(ReasonCode code)
{
    switch (code)
    {
    case ReasonCode::None: return "NONE";
    case ReasonCode::ProbeFailed: return "PROBE_FAILED";
    case ReasonCode::ManifestMismatch: return "MANIFEST_MISMATCH";
    case ReasonCode::BodyStateStale: return "BODY_STATE_STALE";
    case ReasonCode::ArmStateStale: return "ARM_STATE_STALE";
    case ReasonCode::ArmTrackingErrorTooHigh: return "ARM_TRACKING_ERROR_TOO_HIGH";
    case ReasonCode::PolicyTimeout: return "POLICY_TIMEOUT";
    case ReasonCode::DdsWriteFail: return "DDS_WRITE_FAIL";
    case ReasonCode::Estop: return "ESTOP";
    case ReasonCode::CommandExpired: return "COMMAND_EXPIRED";
    case ReasonCode::IllegalJointMapping: return "ILLEGAL_JOINT_MAPPING";
    }
    return "UNKNOWN";
}

inline const char* ToString(EventType type)
{
    switch (type)
    {
    case EventType::BootComplete: return "BOOT_COMPLETE";
    case EventType::ProbePass: return "PROBE_PASS";
    case EventType::ProbeFail: return "PROBE_FAIL";
    case EventType::OperatorEnable: return "OPERATOR_ENABLE";
    case EventType::OperatorDisable: return "OPERATOR_DISABLE";
    case EventType::PolicyHealthOk: return "POLICY_HEALTH_OK";
    case EventType::PolicyHealthBad: return "POLICY_HEALTH_BAD";
    case EventType::BodyStateStale: return "BODY_STATE_STALE";
    case EventType::ArmStateStale: return "ARM_STATE_STALE";
    case EventType::ArmTrackingErrorHigh: return "ARM_TRACKING_ERROR_HIGH";
    case EventType::Estop: return "ESTOP";
    case EventType::SoftStopRequest: return "SOFT_STOP_REQUEST";
    case EventType::FaultReset: return "FAULT_RESET";
    }
    return "UNKNOWN";
}

} // namespace Go2X5Supervisor

#endif // RL_SAR_RUNTIME_SUPERVISOR_GO2_X5_SUPERVISOR_TYPES_HPP
