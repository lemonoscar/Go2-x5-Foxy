
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/safety_guard.hpp"
#include <mutex>

// ============================================================================
// State Capture and Apply Functions
// ============================================================================

Go2X5ArmRuntime::CommandState RL_Real_Go2X5::CaptureArmCommandStateLocked() const
{
    Go2X5ArmRuntime::CommandState s;
    s.arm_command_size = arm_command_size; s.arm_hold_enabled = arm_hold_enabled;
    s.arm_topic_command_received = arm_topic_command_received;
    s.arm_command_initialized = arm_command_initialized;
    s.arm_command_smoothing_ticks = arm_command_smoothing_ticks;
    s.arm_command_smoothing_counter = arm_command_smoothing_counter;
    s.arm_joint_command_latest = arm_joint_command_latest;
    s.arm_topic_command_latest = arm_topic_command_latest;
    s.arm_hold_position = arm_hold_position;
    s.arm_command_smoothing_start = arm_command_smoothing_start;
    s.arm_command_smoothing_target = arm_command_smoothing_target;
    s.arm_command_smoothed = arm_command_smoothed;
    return s;
}

void RL_Real_Go2X5::ApplyArmCommandStateLocked(const Go2X5ArmRuntime::CommandState& state)
{
    this->arm_command_size = state.arm_command_size;
    this->arm_hold_enabled = state.arm_hold_enabled;
    this->arm_topic_command_received = state.arm_topic_command_received;
    this->arm_command_initialized = state.arm_command_initialized;
    this->arm_command_smoothing_ticks = state.arm_command_smoothing_ticks;
    this->arm_command_smoothing_counter = state.arm_command_smoothing_counter;
    this->arm_joint_command_latest = state.arm_joint_command_latest;
    this->arm_topic_command_latest = state.arm_topic_command_latest;
    this->arm_hold_position = state.arm_hold_position;
    this->arm_command_smoothing_start = state.arm_command_smoothing_start;
    this->arm_command_smoothing_target = state.arm_command_smoothing_target;
    this->arm_command_smoothed = state.arm_command_smoothed;
}

Go2X5ArmBridgeRuntime::RuntimeState RL_Real_Go2X5::CaptureArmBridgeRuntimeStateLocked() const
{
    Go2X5ArmBridgeRuntime::RuntimeState state;
    state.joint_count = this->arm_joint_count;
    state.require_state = this->arm_bridge_require_state;
    state.require_live_state = this->arm_bridge_require_live_state;
    state.state_timeout_sec = this->arm_bridge_state_timeout_sec;
    state.state_valid = this->arm_bridge_state_valid;
    state.state_from_backend = this->arm_bridge_state_from_backend;
    state.state_timeout_warned = this->arm_bridge_state_timeout_warned;
    state.shadow_mode_warned = this->arm_bridge_shadow_mode_warned;
    state.state_stream_logged = this->arm_bridge_state_stream_logged;
    state.state_stamp = this->arm_bridge_state_stamp;
    state.external_state_q = this->arm_external_state_q;
    state.external_state_dq = this->arm_external_state_dq;
    state.external_state_tau = this->arm_external_state_tau;
    state.shadow_q = this->arm_external_shadow_q;
    state.shadow_dq = this->arm_external_shadow_dq;
    return state;
}

void RL_Real_Go2X5::ApplyArmBridgeRuntimeStateLocked(const Go2X5ArmBridgeRuntime::RuntimeState& state)
{
    this->arm_joint_count = state.joint_count;
    this->arm_bridge_require_state = state.require_state;
    this->arm_bridge_require_live_state = state.require_live_state;
    this->arm_bridge_state_timeout_sec = state.state_timeout_sec;
    this->arm_bridge_state_valid = state.state_valid;
    this->arm_bridge_state_from_backend = state.state_from_backend;
    this->arm_bridge_state_timeout_warned = state.state_timeout_warned;
    this->arm_bridge_shadow_mode_warned = state.shadow_mode_warned;
    this->arm_bridge_state_stream_logged = state.state_stream_logged;
    this->arm_bridge_state_stamp = state.state_stamp;
    this->arm_external_state_q = state.external_state_q;
    this->arm_external_state_dq = state.external_state_dq;
    this->arm_external_state_tau = state.external_state_tau;
    this->arm_external_shadow_q = state.shadow_q;
    this->arm_external_shadow_dq = state.shadow_dq;
}

void RL_Real_Go2X5::RestoreArmRuntimeStateLocked(
    const Go2X5ArmRuntime::CommandState& cmd,
    const Go2X5ArmBridgeRuntime::RuntimeState& bridge)
{
    this->ApplyArmCommandStateLocked(cmd);
    this->ApplyArmBridgeRuntimeStateLocked(bridge);
}

bool RL_Real_Go2X5::IsArmBridgeStateFreshLocked() const
{
    Go2X5SafetyGuard::ArmBridgeStateSnapshot state_snapshot;
    state_snapshot.state_valid = this->arm_bridge_state_valid;
    state_snapshot.require_live_state = this->arm_bridge_require_live_state;
    state_snapshot.state_from_backend = this->arm_bridge_state_from_backend;
    state_snapshot.state_timeout_sec = this->arm_bridge_state_timeout_sec;
    state_snapshot.state_stamp = this->arm_bridge_state_stamp;
    return Go2X5SafetyGuard::IsArmBridgeStateFresh(state_snapshot);
}

void RL_Real_Go2X5::RestoreArmRuntimeStateLocked(
    const Go2X5ControlLogic::ArmRuntimeStateSnapshot& snapshot)
{
    auto state = this->CaptureArmCommandStateLocked();
    Go2X5ArmRuntime::RestoreSnapshotIfCompatible(&state, snapshot);
    this->ApplyArmCommandStateLocked(state);
}

Go2X5ControlLogic::ArmRuntimeStateSnapshot RL_Real_Go2X5::CaptureArmRuntimeStateLocked() const
{
    return Go2X5ArmRuntime::CaptureSnapshot(this->CaptureArmCommandStateLocked());
}
