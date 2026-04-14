
#ifndef GO2_X5_STATE_MANAGER_HPP
#define GO2_X5_STATE_MANAGER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

namespace Go2X5State {

/**
 * @brief Thread-safe state for Go2-X5 robot
 *
 * This structure contains all runtime state that needs to be accessed
 * from multiple threads in a thread-safe manner.
 */
struct ThreadSafeState {
    // Atomic flags for frequently accessed boolean state
    std::atomic<bool> arm_bridge_connected{false};
    std::atomic<bool> in_rl_locomotion{false};
    std::atomic<bool> arm_hold_enabled{true};
    std::atomic<bool> arm_lock{false};
    std::atomic<bool> safe_shutdown_active{false};

    // Configuration (read-only after init, use shared_mutex for writes)
    mutable std::shared_mutex config_mutex;
    int num_dofs = 18;
    int arm_joint_start_index = 12;
    int arm_joint_count = 6;
    std::string arm_control_mode = "split";

    // Arm command state (use mutex for all access)
    mutable std::mutex arm_command_mutex;
    std::vector<float> arm_hold_position;
    std::vector<float> arm_command_smoothed;
    std::vector<float> arm_joint_command_latest;
    std::vector<float> arm_topic_command_latest;
    bool arm_topic_command_received = false;
    int arm_command_smoothing_counter = 0;
    int arm_command_smoothing_ticks = 0;
    std::vector<float> arm_command_smoothing_start;
    std::vector<float> arm_command_smoothing_target;

    // Arm bridge state (use mutex for all access)
    mutable std::mutex arm_bridge_state_mutex;
    std::vector<float> arm_bridge_state_q;
    std::vector<float> arm_bridge_state_dq;
    std::vector<float> arm_bridge_state_tau;
    std::chrono::steady_clock::time_point arm_bridge_state_stamp;
    bool arm_bridge_state_valid = false;
    bool arm_bridge_state_from_backend = false;

};

/**
 * @brief Immutable snapshot of the robot state
 *
 * Used for consistent state reads without holding locks.
 */
struct StateSnapshot {
    // Flags
    bool arm_bridge_connected = false;
    bool in_rl_locomotion = false;
    bool arm_hold_enabled = true;
    bool arm_lock = false;

    // Config
    int num_dofs = 18;
    int arm_joint_start_index = 12;
    int arm_joint_count = 6;
    std::string arm_control_mode = "split";

    // Arm command state
    std::vector<float> arm_hold_position;
    std::vector<float> arm_command_smoothed;
    bool arm_topic_command_received = false;
    int arm_command_smoothing_counter = 0;

    // Arm bridge state
    std::vector<float> arm_bridge_state_q;
    std::vector<float> arm_bridge_state_dq;
    std::vector<float> arm_bridge_state_tau;
    std::chrono::steady_clock::time_point arm_bridge_state_stamp;
    bool arm_bridge_state_valid = false;
    bool arm_bridge_state_from_backend = false;

    // Timestamp
    std::chrono::steady_clock::time_point snapshot_time;
};

/**
 * @brief State manager for Go2-X5 robot
 *
 * Provides thread-safe access to all robot state with clear
 * semantics for read vs write operations.
 */
class StateManager {
public:
    StateManager();
    explicit StateManager(int num_dofs, int arm_joint_count = 6);

    // ========================================================================
    // Atomic Flag Access (Lock-free, fastest)
    // ========================================================================

    bool IsArmBridgeConnected() const;
    void SetArmBridgeConnected(bool connected);

    bool IsInRLLocomotion() const;
    void SetInRLLocomotion(bool in_rl);

    bool IsArmHoldEnabled() const;
    void SetArmHoldEnabled(bool enabled);

    bool IsArmLock() const;
    void SetArmLock(bool locked);

    bool IsSafeShutdownActive() const;
    void SetSafeShutdownActive(bool active);

    // ========================================================================
    // Configuration Access (Read-heavy, use shared lock)
    // ========================================================================

    int GetNumDofs() const;
    int GetArmJointStartIndex() const;
    int GetArmJointCount() const;
    std::string GetArmControlMode() const;

    void SetConfig(int num_dofs, int arm_joint_start_index, int arm_joint_count,
                  const std::string& arm_control_mode);

    // ========================================================================
    // Arm Command State Access
    // ========================================================================

    // Get arm hold position
    std::vector<float> GetArmHoldPosition() const;
    void SetArmHoldPosition(const std::vector<float>& position);

    // Get arm command smoothed
    std::vector<float> GetArmCommandSmoothed() const;
    void SetArmCommandSmoothed(const std::vector<float>& smoothed);

    // Get arm joint command latest
    std::vector<float> GetArmJointCommandLatest() const;
    void SetArmJointCommandLatest(const std::vector<float>& command);

    // Get arm topic command
    std::vector<float> GetArmTopicCommandLatest() const;
    void SetArmTopicCommandLatest(const std::vector<float>& command);

    bool HasArmTopicCommand() const;
    void SetArmTopicCommandReceived(bool received);

    // Smoothing state
    int GetArmCommandSmoothingCounter() const;
    void SetArmCommandSmoothingCounter(int counter);
    int GetArmCommandSmoothingTicks() const;
    void SetArmCommandSmoothingTicks(int ticks);
    std::vector<float> GetArmCommandSmoothingStart() const;
    void SetArmCommandSmoothingStart(const std::vector<float>& start);
    std::vector<float> GetArmCommandSmoothingTarget() const;
    void SetArmCommandSmoothingTarget(const std::vector<float>& target);

    // ========================================================================
    // Arm Bridge State Access
    // ========================================================================

    // Get full arm bridge state
    bool GetArmBridgeState(std::vector<float>* q, std::vector<float>* dq,
                          std::vector<float>* tau) const;

    // Set arm bridge state
    void SetArmBridgeState(const std::vector<float>& q, const std::vector<float>& dq,
                          const std::vector<float>& tau, bool from_backend);

    // Check if arm bridge state is fresh
    bool IsArmBridgeStateFresh(double max_age_ms = 250.0) const;

    // Get arm bridge state validity
    bool IsArmBridgeStateValid() const;
    void SetArmBridgeStateValid(bool valid);

    // Get arm bridge state timestamp
    std::chrono::steady_clock::time_point GetArmBridgeStateStamp() const;

    // ========================================================================
    // Snapshot Operations
    // ========================================================================

    /**
     * @brief Capture a consistent snapshot of all state
     * @return StateSnapshot containing all state at the moment of the call
     */
    StateSnapshot CaptureSnapshot() const;

    /**
     * @brief Restore state from a snapshot
     * @param snapshot The snapshot to restore from
     *
     * Note: This does NOT restore atomic flags, which should be
     * managed separately during state transitions.
     */
    void RestoreFromSnapshot(const StateSnapshot& snapshot);

    // ========================================================================
    // State Reset
    // ========================================================================

    /**
     * @brief Reset all state to default values
     */
    void Reset();

    /**
     * @brief Reset arm command state
     */
    void ResetArmCommandState();

    /**
     * @brief Reset arm bridge state
     */
    void ResetArmBridgeState();

    // Allow ScopedStateUpdate to access private members
    friend class ScopedStateUpdate;

private:
    ThreadSafeState state_;
};

/**
 * @brief Scoped state lock for atomic multi-field updates
 *
 * This class provides a way to atomically update multiple state fields
 * while ensuring consistent reads from other threads.
 */
class ScopedStateUpdate {
public:
    explicit ScopedStateUpdate(StateManager& manager);
    ~ScopedStateUpdate();

    // Non-copyable
    ScopedStateUpdate(const ScopedStateUpdate&) = delete;
    ScopedStateUpdate& operator=(const ScopedStateUpdate&) = delete;

    // Provide direct access to state for batch updates
    // Note: This bypasses the normal thread-safe accessors, use with care!
    ThreadSafeState& State() { return manager_.state_; }

private:
    StateManager& manager_;
};

/**
 * @brief Scoped snapshot for consistent reads
 */
class ScopedStateRead {
public:
    explicit ScopedStateRead(const StateManager& manager);
    ~ScopedStateRead() = default;

    const StateSnapshot& Snapshot() const { return snapshot_; }

private:
    StateSnapshot snapshot_;
};

} // namespace Go2X5State

#endif // GO2_X5_STATE_MANAGER_HPP
