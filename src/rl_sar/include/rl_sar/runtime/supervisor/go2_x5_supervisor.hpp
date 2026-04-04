#ifndef RL_SAR_RUNTIME_SUPERVISOR_GO2_X5_SUPERVISOR_HPP
#define RL_SAR_RUNTIME_SUPERVISOR_GO2_X5_SUPERVISOR_HPP

#include <cstdint>
#include <vector>

#include "rl_sar/runtime/supervisor/go2_x5_supervisor_types.hpp"

namespace Go2X5Supervisor
{

class Supervisor
{
public:
    explicit Supervisor(const Config& config = Config{});

    void Reset();
    void SetConfig(const Config& config);

    const Config& config() const noexcept;
    Mode mode() const noexcept;
    ReasonCode last_reason_code() const noexcept;
    uint64_t mode_enter_monotonic_ns() const noexcept;
    uint64_t last_transition_seq() const noexcept;
    uint64_t heartbeat_monotonic_ns() const noexcept;

    void NoteHeartbeat(uint64_t source_monotonic_ns);
    uint64_t HeartbeatAgeUs(uint64_t now_monotonic_ns) const noexcept;
    bool HeartbeatFresh(uint64_t now_monotonic_ns) const noexcept;

    WatchdogStatus EvaluateWatchdog(const WatchdogInput& input) const;
    TransitionResult Step(const WatchdogInput& input);

    bool CanEnterReady(const WatchdogInput& input) const;
    bool IsInMode(Mode expected) const noexcept;
    bool IsLatchedFault() const noexcept;

    const std::vector<ModeEventRecord>& event_log() const noexcept;
    void ClearEventLog();

    static bool IsFresh(uint64_t age_us, uint64_t limit_us);
    static bool HasSequenceGap(uint64_t current_seq, uint64_t previous_seq);

private:
    TransitionResult TransitionTo(
        Mode next_mode,
        ReasonCode reason_code,
        const WatchdogInput& input,
        const WatchdogStatus& watchdog,
        uint64_t detail_value,
        EventType event_type);

    bool ProbeTimedOut(const WatchdogInput& input) const;
    bool SoftStopTimedOut(const WatchdogInput& input) const;
    bool DegradedTimedOut(const WatchdogInput& input) const;

    Config config_;
    Mode mode_ = Mode::Boot;
    ReasonCode last_reason_code_ = ReasonCode::None;
    uint64_t mode_enter_monotonic_ns_ = 0;
    uint64_t last_transition_seq_ = 0;
    uint64_t heartbeat_monotonic_ns_ = 0;
    bool has_heartbeat_ = false;
    uint64_t last_body_state_seq_ = 0;
    uint64_t last_arm_state_seq_ = 0;
    uint64_t last_policy_seq_ = 0;
    bool has_body_state_seq_ = false;
    bool has_arm_state_seq_ = false;
    bool has_policy_seq_ = false;
    std::vector<ModeEventRecord> event_log_;
};

} // namespace Go2X5Supervisor

#endif // RL_SAR_RUNTIME_SUPERVISOR_GO2_X5_SUPERVISOR_HPP
