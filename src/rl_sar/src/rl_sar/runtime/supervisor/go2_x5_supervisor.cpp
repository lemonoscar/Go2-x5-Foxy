#include "rl_sar/runtime/supervisor/go2_x5_supervisor.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace Go2X5Supervisor
{

namespace
{

uint64_t ElapsedUs(uint64_t now_monotonic_ns, uint64_t start_monotonic_ns)
{
    if (now_monotonic_ns == 0 || start_monotonic_ns == 0 || now_monotonic_ns < start_monotonic_ns)
    {
        return 0;
    }
    return (now_monotonic_ns - start_monotonic_ns) / 1000ULL;
}

bool ShouldUseSequenceGap(uint64_t current_seq, bool has_previous_seq, uint64_t previous_seq)
{
    if (!has_previous_seq || current_seq == 0)
    {
        return false;
    }
    return current_seq != (previous_seq + 1ULL);
}

const char* ResolveEventSource(const EventType event_type, const ReasonCode reason_code)
{
    switch (reason_code)
    {
    case ReasonCode::ManifestMismatch:
        return "manifest";
    case ReasonCode::BodyStateStale:
    case ReasonCode::DdsWriteFail:
        return "body";
    case ReasonCode::ArmStateStale:
    case ReasonCode::ArmTrackingErrorTooHigh:
        return "arm";
    case ReasonCode::PolicyTimeout:
        return "policy";
    case ReasonCode::Estop:
        return "operator";
    case ReasonCode::ProbeFailed:
    case ReasonCode::CommandExpired:
    case ReasonCode::IllegalJointMapping:
    case ReasonCode::None:
        break;
    }

    switch (event_type)
    {
    case EventType::OperatorEnable:
    case EventType::OperatorDisable:
    case EventType::Estop:
    case EventType::SoftStopRequest:
    case EventType::FaultReset:
        return "operator";
    case EventType::PolicyHealthOk:
    case EventType::PolicyHealthBad:
        return "policy";
    case EventType::BodyStateStale:
        return "body";
    case EventType::ArmStateStale:
    case EventType::ArmTrackingErrorHigh:
        return "arm";
    case EventType::BootComplete:
    case EventType::ProbePass:
    case EventType::ProbeFail:
        return "supervisor";
    }
    return "supervisor";
}

} // namespace

Supervisor::Supervisor(const Config& config)
    : config_(config)
{
}

void Supervisor::Reset()
{
    this->mode_ = Mode::Boot;
    this->last_reason_code_ = ReasonCode::None;
    this->mode_enter_monotonic_ns_ = 0;
    this->last_transition_seq_ = 0;
    this->heartbeat_monotonic_ns_ = 0;
    this->has_heartbeat_ = false;
    this->last_body_state_seq_ = 0;
    this->last_arm_state_seq_ = 0;
    this->last_policy_seq_ = 0;
    this->has_body_state_seq_ = false;
    this->has_arm_state_seq_ = false;
    this->has_policy_seq_ = false;
    this->event_log_.clear();
}

void Supervisor::SetConfig(const Config& config)
{
    this->config_ = config;
}

const Config& Supervisor::config() const noexcept
{
    return this->config_;
}

Mode Supervisor::mode() const noexcept
{
    return this->mode_;
}

ReasonCode Supervisor::last_reason_code() const noexcept
{
    return this->last_reason_code_;
}

uint64_t Supervisor::mode_enter_monotonic_ns() const noexcept
{
    return this->mode_enter_monotonic_ns_;
}

uint64_t Supervisor::last_transition_seq() const noexcept
{
    return this->last_transition_seq_;
}

uint64_t Supervisor::heartbeat_monotonic_ns() const noexcept
{
    return this->heartbeat_monotonic_ns_;
}

void Supervisor::NoteHeartbeat(uint64_t source_monotonic_ns)
{
    this->heartbeat_monotonic_ns_ = source_monotonic_ns;
    this->has_heartbeat_ = true;
}

uint64_t Supervisor::HeartbeatAgeUs(uint64_t now_monotonic_ns) const noexcept
{
    if (!this->has_heartbeat_)
    {
        return std::numeric_limits<uint64_t>::max();
    }
    return ElapsedUs(now_monotonic_ns, this->heartbeat_monotonic_ns_);
}

bool Supervisor::HeartbeatFresh(uint64_t now_monotonic_ns) const noexcept
{
    return this->HeartbeatAgeUs(now_monotonic_ns) <= this->config_.probe_window_us;
}

bool Supervisor::IsFresh(uint64_t age_us, uint64_t limit_us)
{
    return age_us <= limit_us;
}

bool Supervisor::HasSequenceGap(uint64_t current_seq, uint64_t previous_seq)
{
    return ShouldUseSequenceGap(current_seq, previous_seq != 0ULL, previous_seq);
}

bool Supervisor::IsInMode(Mode expected) const noexcept
{
    return this->mode_ == expected;
}

bool Supervisor::IsLatchedFault() const noexcept
{
    return this->mode_ == Mode::FaultLatched;
}

const std::vector<ModeEventRecord>& Supervisor::event_log() const noexcept
{
    return this->event_log_;
}

void Supervisor::ClearEventLog()
{
    this->event_log_.clear();
}

bool Supervisor::ProbeTimedOut(const WatchdogInput& input) const
{
    return ElapsedUs(input.now_monotonic_ns, this->mode_enter_monotonic_ns_) > this->config_.probe_window_us;
}

bool Supervisor::SoftStopTimedOut(const WatchdogInput& input) const
{
    return ElapsedUs(input.now_monotonic_ns, this->mode_enter_monotonic_ns_) > this->config_.soft_stop_duration_us;
}

bool Supervisor::DegradedTimedOut(const WatchdogInput& input) const
{
    return ElapsedUs(input.now_monotonic_ns, this->mode_enter_monotonic_ns_) > this->config_.degraded_timeout_us;
}

WatchdogStatus Supervisor::EvaluateWatchdog(const WatchdogInput& input) const
{
    WatchdogStatus status;
    status.now_monotonic_ns = input.now_monotonic_ns;
    status.heartbeat_age_us = this->HeartbeatAgeUs(input.now_monotonic_ns);
    status.body_state_age_us = input.body_state_age_us;
    status.arm_state_age_us = input.arm_state_age_us;
    status.policy_age_us = input.policy_age_us;
    status.arm_tracking_error_age_us = input.arm_tracking_error_age_us;
    status.heartbeat_fresh = status.heartbeat_age_us <= this->config_.probe_window_us;

    status.manifest_valid = input.manifest_valid;
    status.body_state_fresh = IsFresh(input.body_state_age_us, this->config_.body_state_stale_us);
    status.arm_state_fresh = IsFresh(input.arm_state_age_us, this->config_.arm_state_stale_us);
    status.policy_fresh = IsFresh(input.policy_age_us, this->config_.policy_stale_us);
    status.policy_health_ok = input.policy_health_ok && !input.policy_health_bad;
    status.body_dds_write_ok = input.body_dds_write_ok;
    status.arm_backend_valid = input.arm_backend_valid;
    status.arm_tracking_error_high =
        input.arm_tracking_error_high ||
        (input.arm_tracking_error_age_us > this->config_.arm_tracking_error_window_us);
    status.arm_tracking_error_window_exceeded =
        input.arm_tracking_error_age_us > this->config_.arm_tracking_error_window_us;

    status.body_seq_gap =
        ShouldUseSequenceGap(input.body_state_seq, this->has_body_state_seq_, this->last_body_state_seq_);
    status.arm_seq_gap =
        ShouldUseSequenceGap(input.arm_state_seq, this->has_arm_state_seq_, this->last_arm_state_seq_);
    status.policy_seq_gap =
        ShouldUseSequenceGap(input.policy_seq, this->has_policy_seq_, this->last_policy_seq_);
    status.seq_gap_count =
        static_cast<uint32_t>(status.body_seq_gap) +
        static_cast<uint32_t>(status.arm_seq_gap) +
        static_cast<uint32_t>(status.policy_seq_gap);

    status.ready_gate_ok =
        (!this->config_.require_manifest_valid || status.manifest_valid) &&
        status.heartbeat_fresh &&
        status.body_state_fresh &&
        status.arm_state_fresh;

    status.active_gate_ok =
        status.ready_gate_ok &&
        status.policy_fresh &&
        status.policy_health_ok &&
        status.body_dds_write_ok &&
        status.arm_backend_valid &&
        !status.arm_tracking_error_high;

    return status;
}

bool Supervisor::CanEnterReady(const WatchdogInput& input) const
{
    const WatchdogStatus status = this->EvaluateWatchdog(input);
    return status.ready_gate_ok;
}

TransitionResult Supervisor::TransitionTo(
    Mode next_mode,
    ReasonCode reason_code,
    const WatchdogInput& input,
    const WatchdogStatus& watchdog,
    uint64_t detail_value,
    EventType event_type)
{
    TransitionResult result;
    result.previous_mode = this->mode_;
    result.current_mode = next_mode;
    result.reason_code = reason_code;
    result.trigger_seq = this->last_transition_seq_;
    result.source_monotonic_ns = input.now_monotonic_ns;
    result.detail_value = detail_value;
    result.watchdog = watchdog;
    result.mode_changed = (next_mode != this->mode_);

    if (!result.mode_changed)
    {
        return result;
    }

    this->mode_ = next_mode;
    this->last_reason_code_ = reason_code;
    this->mode_enter_monotonic_ns_ = input.now_monotonic_ns;
    ++this->last_transition_seq_;
    result.trigger_seq = this->last_transition_seq_;
    result.current_mode = this->mode_;

    this->event_log_.push_back(ModeEventRecord{
        result.previous_mode,
        result.current_mode,
        reason_code,
        result.trigger_seq,
        input.now_monotonic_ns,
        detail_value,
        event_type,
        ResolveEventSource(event_type, reason_code),
    });

    return result;
}

TransitionResult Supervisor::Step(const WatchdogInput& input)
{
    const WatchdogStatus watchdog = this->EvaluateWatchdog(input);

    if (input.has_body_state_seq)
    {
        this->last_body_state_seq_ = input.body_state_seq;
        this->has_body_state_seq_ = true;
    }
    if (input.has_arm_state_seq)
    {
        this->last_arm_state_seq_ = input.arm_state_seq;
        this->has_arm_state_seq_ = true;
    }
    if (input.has_policy_seq)
    {
        this->last_policy_seq_ = input.policy_seq;
        this->has_policy_seq_ = true;
    }

    if (input.estop)
    {
        return this->TransitionTo(
            Mode::FaultLatched,
            ReasonCode::Estop,
            input,
            watchdog,
            0,
            EventType::Estop);
    }

    if (this->config_.require_manifest_valid && !watchdog.manifest_valid)
    {
        return this->TransitionTo(
            Mode::FaultLatched,
            ReasonCode::ManifestMismatch,
            input,
            watchdog,
            0,
            EventType::ProbeFail);
    }

    switch (this->mode_)
    {
    case Mode::Boot:
        if (input.boot_complete || input.config_loaded)
        {
            return this->TransitionTo(
                Mode::Probe,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::BootComplete);
        }
        break;

    case Mode::Probe:
        if (input.probe_fail || this->ProbeTimedOut(input))
        {
            const uint64_t detail = this->HeartbeatAgeUs(input.now_monotonic_ns);
            return this->TransitionTo(
                Mode::FaultLatched,
                ReasonCode::ProbeFailed,
                input,
                watchdog,
                detail,
                EventType::ProbeFail);
        }
        if (input.probe_pass)
        {
            return this->TransitionTo(
                Mode::Passive,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::ProbePass);
        }
        break;

    case Mode::Passive:
        if (input.probe_fail)
        {
            return this->TransitionTo(
                Mode::FaultLatched,
                ReasonCode::ProbeFailed,
                input,
                watchdog,
                0,
                EventType::ProbeFail);
        }
        if (this->CanEnterReady(input))
        {
            return this->TransitionTo(
                Mode::Ready,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::PolicyHealthOk);
        }
        break;

    case Mode::Ready:
        if (input.soft_stop_request)
        {
            return this->TransitionTo(
                Mode::SoftStop,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::SoftStopRequest);
        }
        if (input.operator_disable)
        {
            return this->TransitionTo(
                Mode::Passive,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::OperatorDisable);
        }
        if (input.manual_arm_request)
        {
            return this->TransitionTo(
                Mode::ManualArm,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::OperatorEnable);
        }
        if (input.operator_enable && watchdog.active_gate_ok)
        {
            return this->TransitionTo(
                Mode::RlDogOnlyActive,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::OperatorEnable);
        }
        break;

    case Mode::ManualArm:
        if (input.soft_stop_request || input.operator_disable)
        {
            return this->TransitionTo(
                Mode::SoftStop,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::SoftStopRequest);
        }
        if (!input.manual_arm_request && this->CanEnterReady(input))
        {
            return this->TransitionTo(
                Mode::Ready,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::OperatorDisable);
        }
        break;

    case Mode::RlDogOnlyActive:
        if (input.soft_stop_request || input.operator_disable)
        {
            return this->TransitionTo(
                Mode::SoftStop,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::SoftStopRequest);
        }
        if (!watchdog.body_dds_write_ok || !watchdog.body_state_fresh)
        {
            const ReasonCode reason = !watchdog.body_dds_write_ok ? ReasonCode::DdsWriteFail : ReasonCode::BodyStateStale;
            return this->TransitionTo(
                Mode::DegradedBody,
                reason,
                input,
                watchdog,
                watchdog.body_state_age_us,
                EventType::BodyStateStale);
        }
        if (!watchdog.arm_state_fresh || !watchdog.arm_backend_valid)
        {
            const ReasonCode reason = !watchdog.arm_state_fresh ? ReasonCode::ArmStateStale : ReasonCode::ArmStateStale;
            return this->TransitionTo(
                Mode::DegradedArm,
                reason,
                input,
                watchdog,
                watchdog.arm_state_age_us,
                EventType::ArmStateStale);
        }
        if (watchdog.arm_tracking_error_high)
        {
            return this->TransitionTo(
                Mode::DegradedArm,
                ReasonCode::ArmTrackingErrorTooHigh,
                input,
                watchdog,
                input.arm_tracking_error_age_us,
                EventType::ArmTrackingErrorHigh);
        }
        if (!watchdog.policy_fresh || !watchdog.policy_health_ok)
        {
            return this->TransitionTo(
                Mode::SoftStop,
                ReasonCode::PolicyTimeout,
                input,
                watchdog,
                watchdog.policy_fresh ? 0ULL : input.policy_age_us,
                EventType::PolicyHealthBad);
        }
        break;

    case Mode::DegradedArm:
        if (this->DegradedTimedOut(input))
        {
            return this->TransitionTo(
                Mode::SoftStop,
                ReasonCode::ArmStateStale,
                input,
                watchdog,
                this->config_.degraded_timeout_us,
                EventType::SoftStopRequest);
        }
        if (input.allow_recover && watchdog.arm_state_fresh && watchdog.arm_backend_valid && !watchdog.arm_tracking_error_high)
        {
            return this->TransitionTo(
                Mode::Ready,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::ArmStateStale);
        }
        break;

    case Mode::DegradedBody:
        if (this->DegradedTimedOut(input))
        {
            return this->TransitionTo(
                Mode::SoftStop,
                ReasonCode::BodyStateStale,
                input,
                watchdog,
                this->config_.degraded_timeout_us,
                EventType::SoftStopRequest);
        }
        if (input.allow_recover && watchdog.body_state_fresh && watchdog.body_dds_write_ok)
        {
            return this->TransitionTo(
                Mode::Ready,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::BodyStateStale);
        }
        break;

    case Mode::SoftStop:
        if (this->SoftStopTimedOut(input))
        {
            return this->TransitionTo(
                Mode::Passive,
                ReasonCode::None,
                input,
                watchdog,
                this->config_.soft_stop_duration_us,
                EventType::SoftStopRequest);
        }
        break;

    case Mode::FaultLatched:
        if ((this->config_.fault_latched_requires_manual_reset &&
                input.fault_reset && input.probe_pass && watchdog.manifest_valid) ||
            (!this->config_.fault_latched_requires_manual_reset &&
                input.probe_pass && watchdog.manifest_valid))
        {
            return this->TransitionTo(
                Mode::Passive,
                ReasonCode::None,
                input,
                watchdog,
                0,
                EventType::FaultReset);
        }
        break;
    }

    TransitionResult result;
    result.previous_mode = this->mode_;
    result.current_mode = this->mode_;
    result.reason_code = this->last_reason_code_;
    result.source_monotonic_ns = input.now_monotonic_ns;
    result.trigger_seq = this->last_transition_seq_;
    result.watchdog = watchdog;
    return result;
}

} // namespace Go2X5Supervisor
