
#include "rl_sar/go2x5/state_manager.hpp"

#include <algorithm>
#include <cstring>

namespace Go2X5State {

// ============================================================================
// StateManager Implementation
// ============================================================================

StateManager::StateManager() {
    state_.num_dofs = 18;
    state_.arm_joint_start_index = 12;
    state_.arm_joint_count = 6;
    state_.arm_control_mode = "split";

    // Initialize arm vectors
    state_.arm_hold_position.assign(6, 0.0f);
    state_.arm_command_smoothed.assign(6, 0.0f);
    state_.arm_joint_command_latest.assign(6, 0.0f);
    state_.arm_topic_command_latest.assign(6, 0.0f);
    state_.arm_command_smoothing_start.assign(6, 0.0f);
    state_.arm_command_smoothing_target.assign(6, 0.0f);

    state_.arm_bridge_state_q.assign(6, 0.0f);
    state_.arm_bridge_state_dq.assign(6, 0.0f);
    state_.arm_bridge_state_tau.assign(6, 0.0f);
}

StateManager::StateManager(int num_dofs, int arm_joint_count)
    : StateManager() {
    state_.num_dofs = num_dofs;
    state_.arm_joint_count = arm_joint_count;
    state_.arm_joint_start_index = std::max(0, num_dofs - arm_joint_count);

    // Reinitialize vectors with correct size
    state_.arm_hold_position.assign(arm_joint_count, 0.0f);
    state_.arm_command_smoothed.assign(arm_joint_count, 0.0f);
    state_.arm_joint_command_latest.assign(arm_joint_count, 0.0f);
    state_.arm_topic_command_latest.assign(arm_joint_count, 0.0f);
    state_.arm_command_smoothing_start.assign(arm_joint_count, 0.0f);
    state_.arm_command_smoothing_target.assign(arm_joint_count, 0.0f);

    state_.arm_bridge_state_q.assign(arm_joint_count, 0.0f);
    state_.arm_bridge_state_dq.assign(arm_joint_count, 0.0f);
    state_.arm_bridge_state_tau.assign(arm_joint_count, 0.0f);
}

// ============================================================================
// Atomic Flag Access
// ============================================================================

bool StateManager::IsArmBridgeConnected() const {
    return state_.arm_bridge_connected.load(std::memory_order_relaxed);
}

void StateManager::SetArmBridgeConnected(bool connected) {
    state_.arm_bridge_connected.store(connected, std::memory_order_relaxed);
}

bool StateManager::IsInRLLocomotion() const {
    return state_.in_rl_locomotion.load(std::memory_order_relaxed);
}

void StateManager::SetInRLLocomotion(bool in_rl) {
    state_.in_rl_locomotion.store(in_rl, std::memory_order_relaxed);
}

bool StateManager::IsArmHoldEnabled() const {
    return state_.arm_hold_enabled.load(std::memory_order_relaxed);
}

void StateManager::SetArmHoldEnabled(bool enabled) {
    state_.arm_hold_enabled.store(enabled, std::memory_order_relaxed);
}

bool StateManager::IsArmLock() const {
    return state_.arm_lock.load(std::memory_order_relaxed);
}

void StateManager::SetArmLock(bool locked) {
    state_.arm_lock.store(locked, std::memory_order_relaxed);
}

bool StateManager::IsSafeShutdownActive() const {
    return state_.safe_shutdown_active.load(std::memory_order_acquire);
}

void StateManager::SetSafeShutdownActive(bool active) {
    state_.safe_shutdown_active.store(active, std::memory_order_release);
}

// ============================================================================
// Configuration Access
// ============================================================================

int StateManager::GetNumDofs() const {
    std::shared_lock<std::shared_mutex> lock(state_.config_mutex);
    return state_.num_dofs;
}

int StateManager::GetArmJointStartIndex() const {
    std::shared_lock<std::shared_mutex> lock(state_.config_mutex);
    return state_.arm_joint_start_index;
}

int StateManager::GetArmJointCount() const {
    std::shared_lock<std::shared_mutex> lock(state_.config_mutex);
    return state_.arm_joint_count;
}

std::string StateManager::GetArmControlMode() const {
    std::shared_lock<std::shared_mutex> lock(state_.config_mutex);
    return state_.arm_control_mode;
}

void StateManager::SetConfig(int num_dofs, int arm_joint_start_index,
                           int arm_joint_count, const std::string& arm_control_mode) {
    std::unique_lock<std::shared_mutex> lock(state_.config_mutex);
    state_.num_dofs = num_dofs;
    state_.arm_joint_start_index = arm_joint_start_index;
    state_.arm_joint_count = arm_joint_count;
    state_.arm_control_mode = arm_control_mode;
}

// ============================================================================
// Arm Command State Access
// ============================================================================

std::vector<float> StateManager::GetArmHoldPosition() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_hold_position;
}

void StateManager::SetArmHoldPosition(const std::vector<float>& position) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    if (position.size() == static_cast<size_t>(state_.arm_joint_count)) {
        state_.arm_hold_position = position;
    }
}

std::vector<float> StateManager::GetArmCommandSmoothed() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_command_smoothed;
}

void StateManager::SetArmCommandSmoothed(const std::vector<float>& smoothed) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    if (smoothed.size() == static_cast<size_t>(state_.arm_joint_count)) {
        state_.arm_command_smoothed = smoothed;
    }
}

std::vector<float> StateManager::GetArmJointCommandLatest() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_joint_command_latest;
}

void StateManager::SetArmJointCommandLatest(const std::vector<float>& command) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    if (command.size() == static_cast<size_t>(state_.arm_joint_count)) {
        state_.arm_joint_command_latest = command;
    }
}

std::vector<float> StateManager::GetArmTopicCommandLatest() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_topic_command_latest;
}

void StateManager::SetArmTopicCommandLatest(const std::vector<float>& command) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    if (command.size() == static_cast<size_t>(state_.arm_joint_count)) {
        state_.arm_topic_command_latest = command;
    }
}

bool StateManager::HasArmTopicCommand() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_topic_command_received;
}

void StateManager::SetArmTopicCommandReceived(bool received) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    state_.arm_topic_command_received = received;
}

int StateManager::GetArmCommandSmoothingCounter() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_command_smoothing_counter;
}

void StateManager::SetArmCommandSmoothingCounter(int counter) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    state_.arm_command_smoothing_counter = counter;
}

int StateManager::GetArmCommandSmoothingTicks() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_command_smoothing_ticks;
}

void StateManager::SetArmCommandSmoothingTicks(int ticks) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    state_.arm_command_smoothing_ticks = ticks;
}

std::vector<float> StateManager::GetArmCommandSmoothingStart() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_command_smoothing_start;
}

void StateManager::SetArmCommandSmoothingStart(const std::vector<float>& start) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    if (start.size() == static_cast<size_t>(state_.arm_joint_count)) {
        state_.arm_command_smoothing_start = start;
    }
}

std::vector<float> StateManager::GetArmCommandSmoothingTarget() const {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    return state_.arm_command_smoothing_target;
}

void StateManager::SetArmCommandSmoothingTarget(const std::vector<float>& target) {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
    if (target.size() == static_cast<size_t>(state_.arm_joint_count)) {
        state_.arm_command_smoothing_target = target;
    }
}

// ============================================================================
// Arm Bridge State Access
// ============================================================================

bool StateManager::GetArmBridgeState(std::vector<float>* q,
                                     std::vector<float>* dq,
                                     std::vector<float>* tau) const {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);

    if (!state_.arm_bridge_state_valid) {
        return false;
    }

    if (q) *q = state_.arm_bridge_state_q;
    if (dq) *dq = state_.arm_bridge_state_dq;
    if (tau) *tau = state_.arm_bridge_state_tau;

    return true;
}

void StateManager::SetArmBridgeState(const std::vector<float>& q,
                                    const std::vector<float>& dq,
                                    const std::vector<float>& tau,
                                    bool from_backend) {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);

    size_t expected_size = static_cast<size_t>(state_.arm_joint_count);

    if (q.size() == expected_size) {
        state_.arm_bridge_state_q = q;
    }
    if (dq.size() == expected_size) {
        state_.arm_bridge_state_dq = dq;
    }
    if (tau.size() == expected_size) {
        state_.arm_bridge_state_tau = tau;
    }

    state_.arm_bridge_state_from_backend = from_backend;
    state_.arm_bridge_state_stamp = std::chrono::steady_clock::now();
    state_.arm_bridge_state_valid = true;
}

bool StateManager::IsArmBridgeStateFresh(double max_age_ms) const {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);

    if (!state_.arm_bridge_state_valid) {
        return false;
    }

    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration<double, std::milli>(now - state_.arm_bridge_state_stamp).count();

    return age <= max_age_ms;
}

bool StateManager::IsArmBridgeStateValid() const {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);
    return state_.arm_bridge_state_valid;
}

void StateManager::SetArmBridgeStateValid(bool valid) {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);
    state_.arm_bridge_state_valid = valid;
}

std::chrono::steady_clock::time_point StateManager::GetArmBridgeStateStamp() const {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);
    return state_.arm_bridge_state_stamp;
}

// ============================================================================
// Snapshot Operations
// ============================================================================

StateSnapshot StateManager::CaptureSnapshot() const {
    StateSnapshot snapshot;
    snapshot.snapshot_time = std::chrono::steady_clock::now();

    // Atomic flags
    snapshot.arm_bridge_connected = IsArmBridgeConnected();
    snapshot.in_rl_locomotion = IsInRLLocomotion();
    snapshot.arm_hold_enabled = IsArmHoldEnabled();
    snapshot.arm_lock = IsArmLock();

    // Config
    {
        std::shared_lock<std::shared_mutex> lock(state_.config_mutex);
        snapshot.num_dofs = state_.num_dofs;
        snapshot.arm_joint_start_index = state_.arm_joint_start_index;
        snapshot.arm_joint_count = state_.arm_joint_count;
        snapshot.arm_control_mode = state_.arm_control_mode;
    }

    // Arm command state
    {
        std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
        snapshot.arm_hold_position = state_.arm_hold_position;
        snapshot.arm_command_smoothed = state_.arm_command_smoothed;
        snapshot.arm_topic_command_received = state_.arm_topic_command_received;
        snapshot.arm_command_smoothing_counter = state_.arm_command_smoothing_counter;
    }

    // Arm bridge state
    {
        std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);
        snapshot.arm_bridge_state_q = state_.arm_bridge_state_q;
        snapshot.arm_bridge_state_dq = state_.arm_bridge_state_dq;
        snapshot.arm_bridge_state_tau = state_.arm_bridge_state_tau;
        snapshot.arm_bridge_state_stamp = state_.arm_bridge_state_stamp;
        snapshot.arm_bridge_state_valid = state_.arm_bridge_state_valid;
        snapshot.arm_bridge_state_from_backend = state_.arm_bridge_state_from_backend;
    }

    return snapshot;
}

void StateManager::RestoreFromSnapshot(const StateSnapshot& snapshot) {
    // Note: Atomic flags are NOT restored from snapshots
    // They should be managed separately during state transitions

    // Config
    {
        std::unique_lock<std::shared_mutex> lock(state_.config_mutex);
        state_.num_dofs = snapshot.num_dofs;
        state_.arm_joint_start_index = snapshot.arm_joint_start_index;
        state_.arm_joint_count = snapshot.arm_joint_count;
        state_.arm_control_mode = snapshot.arm_control_mode;
    }

    // Arm command state
    {
        std::lock_guard<std::mutex> lock(state_.arm_command_mutex);
        state_.arm_hold_position = snapshot.arm_hold_position;
        state_.arm_command_smoothed = snapshot.arm_command_smoothed;
        state_.arm_topic_command_received = snapshot.arm_topic_command_received;
        state_.arm_command_smoothing_counter = snapshot.arm_command_smoothing_counter;
    }

    // Arm bridge state
    {
        std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);
        state_.arm_bridge_state_q = snapshot.arm_bridge_state_q;
        state_.arm_bridge_state_dq = snapshot.arm_bridge_state_dq;
        state_.arm_bridge_state_tau = snapshot.arm_bridge_state_tau;
        state_.arm_bridge_state_stamp = snapshot.arm_bridge_state_stamp;
        state_.arm_bridge_state_valid = snapshot.arm_bridge_state_valid;
        state_.arm_bridge_state_from_backend = snapshot.arm_bridge_state_from_backend;
    }

}

// ============================================================================
// State Reset
// ============================================================================

void StateManager::Reset() {
    ResetArmCommandState();
    ResetArmBridgeState();

    SetArmBridgeConnected(false);
    SetInRLLocomotion(false);
    SetArmHoldEnabled(true);
    SetArmLock(false);
    SetSafeShutdownActive(false);

}

void StateManager::ResetArmCommandState() {
    std::lock_guard<std::mutex> lock(state_.arm_command_mutex);

    std::fill(state_.arm_hold_position.begin(), state_.arm_hold_position.end(), 0.0f);
    std::fill(state_.arm_command_smoothed.begin(), state_.arm_command_smoothed.end(), 0.0f);
    std::fill(state_.arm_joint_command_latest.begin(), state_.arm_joint_command_latest.end(), 0.0f);
    std::fill(state_.arm_topic_command_latest.begin(), state_.arm_topic_command_latest.end(), 0.0f);
    std::fill(state_.arm_command_smoothing_start.begin(), state_.arm_command_smoothing_start.end(), 0.0f);
    std::fill(state_.arm_command_smoothing_target.begin(), state_.arm_command_smoothing_target.end(), 0.0f);

    state_.arm_topic_command_received = false;
    state_.arm_command_smoothing_counter = 0;
    state_.arm_command_smoothing_ticks = 0;
}

void StateManager::ResetArmBridgeState() {
    std::lock_guard<std::mutex> lock(state_.arm_bridge_state_mutex);

    std::fill(state_.arm_bridge_state_q.begin(), state_.arm_bridge_state_q.end(), 0.0f);
    std::fill(state_.arm_bridge_state_dq.begin(), state_.arm_bridge_state_dq.end(), 0.0f);
    std::fill(state_.arm_bridge_state_tau.begin(), state_.arm_bridge_state_tau.end(), 0.0f);

    state_.arm_bridge_state_stamp = std::chrono::steady_clock::time_point{};
    state_.arm_bridge_state_valid = false;
    state_.arm_bridge_state_from_backend = false;
}

// ============================================================================
// ScopedStateUpdate Implementation
// ============================================================================

ScopedStateUpdate::ScopedStateUpdate(StateManager& manager)
    : manager_(manager) {}

ScopedStateUpdate::~ScopedStateUpdate() {
    // No automatic unlock needed - we didn't take any locks
    // The state is directly accessible during the scope
}

// ============================================================================
// ScopedStateRead Implementation
// ============================================================================

ScopedStateRead::ScopedStateRead(const StateManager& manager)
    : snapshot_(manager.CaptureSnapshot()) {}

} // namespace Go2X5State
