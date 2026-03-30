
#ifndef GO2_X5_ARM_CONTROLLER_HPP
#define GO2_X5_ARM_CONTROLLER_HPP

#include "rl_sar/go2x5/state/go2_x5_state_manager.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_transport.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/comm/go2_x5_ipc_protocol.hpp"
#include "rl_sar/go2x5/safety/go2_x5_safety_guard.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_runtime.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_bridge_runtime.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <functional>

namespace Go2X5ArmController {

/**
 * @brief Configuration for the arm controller
 */
struct ControllerConfig {
    int arm_joint_count = 6;
    int arm_joint_start_index = 12;
    int arm_command_size = 0;
    std::string arm_control_mode = "unitree";
    float dt = 0.02f;
    int decimation = 1;
    float arm_command_smoothing_time = 0.2f;
    bool arm_split_control_enabled = false;
    bool arm_hold_enabled = true;
    bool arm_bridge_require_state = true;
    bool arm_bridge_require_live_state = true;
    float arm_bridge_state_timeout_sec = 0.25f;

    std::vector<float> arm_hold_pose;
    std::vector<float> arm_key_pose;
    std::vector<float> default_dof_pos;
    std::vector<float> fixed_kp;
    std::vector<float> fixed_kd;
    std::vector<float> arm_joint_lower_limits;
    std::vector<float> arm_joint_upper_limits;
    std::vector<float> whole_body_velocity_limits;
    std::vector<float> whole_body_effort_limits;
    std::vector<float> whole_body_kp_limits;
    std::vector<float> whole_body_kd_limits;
};

/**
 * @brief Arm controller result codes
 */
enum class ControllerResult {
    OK = 0,
    NOT_INITIALIZED = 1,
    INVALID_SIZE = 2,
    LIMIT_EXCEEDED = 3,
    TRANSPORT_ERROR = 4,
    STATE_UNAVAILABLE = 5
};

/**
 * @brief Arm command for sending to the bridge
 */
struct ArmBridgeCommand {
    std::vector<float> q;    // Joint positions
    std::vector<float> dq;   // Joint velocities
    std::vector<float> kp;   // Position gains
    std::vector<float> kd;   // Damping gains
    std::vector<float> tau;  // Feedforward torques

    size_t joint_count() const { return q.size(); }
    bool is_valid() const {
        return q.size() == dq.size() &&
               q.size() == kp.size() &&
               q.size() == kd.size() &&
               q.size() == tau.size();
    }
};

/**
 * @brief Arm controller state snapshot
 */
struct ControllerStateSnapshot {
    // Command state
    std::vector<float> arm_hold_position;
    std::vector<float> arm_command_smoothed;
    std::vector<float> arm_joint_command_latest;
    std::vector<float> arm_topic_command_latest;
    bool arm_topic_command_received = false;
    int arm_command_smoothing_counter = 0;
    int arm_command_smoothing_ticks = 0;

    // Bridge state
    std::vector<float> arm_bridge_state_q;
    std::vector<float> arm_bridge_state_dq;
    std::vector<float> arm_bridge_state_tau;
    bool arm_bridge_state_valid = false;
    bool arm_bridge_state_from_backend = false;
    std::chrono::steady_clock::time_point arm_bridge_state_stamp;
};

/**
 * @brief Main arm controller class
 *
 * Integrates StateManager and Transport to provide complete
 * arm control functionality including:
 * - Command smoothing
 * - Hold position management
 * - Bridge communication
 * - State feedback
 * - Safety limiting
 */
class ArmController {
public:
    ArmController();
    ~ArmController();

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * @brief Initialize the controller with config
     * @param config Configuration structure
     * @param state_manager State manager instance
     * @return ControllerResult
     */
    ControllerResult Initialize(
        const ControllerConfig& config,
        Go2X5State::StateManager& state_manager);

    /**
     * @brief Initialize transport for bridge communication
     * @param cmd_port UDP port for commands
     * @param state_port UDP port for state
     * @param host IP address
     * @return true if successful
     */
    bool InitializeTransport(int cmd_port, int state_port, const std::string& host);

    /**
     * @brief Shutdown the controller
     */
    void Shutdown();

    /**
     * @brief Check if controller is initialized
     */
    bool IsInitialized() const { return initialized_; }

    // ========================================================================
    // Command Interface
    // ========================================================================

    /**
     * @brief Set arm hold target position
     * @param target Target joint positions
     * @param reason Description for logging
     * @return ControllerResult
     */
    ControllerResult SetHoldTarget(const std::vector<float>& target, const char* reason = "Hold");

    /**
     * @brief Update arm from topic command (IPC or ROS)
     * @param data Command data
     * @param context Description for logging
     * @return ControllerResult
     */
    ControllerResult UpdateFromTopicCommand(const std::vector<float>& data, const char* context = "Topic");

    /**
     * @brief Step the arm command smoothing
     * @param desired_command Desired joint positions
     * @return Smoothed command (empty if no smoothing active)
     */
    std::vector<float> StepSmoothing(const std::vector<float>& desired_command);

    /**
     * @brief Get the current smoothed command
     */
    std::vector<float> GetSmoothedCommand() const;

    /**
     * @brief Enable/disable arm hold
     */
    void SetHoldEnabled(bool enabled);

    /**
     * @brief Check if hold is enabled
     */
    bool IsHoldEnabled() const;

    // ========================================================================
    // Bridge Communication
    // ========================================================================

    /**
     * @brief Send command to arm bridge
     * @param cmd Command to send
     * @return ControllerResult
     */
    ControllerResult SendBridgeCommand(const ArmBridgeCommand& cmd);

    /**
     * @brief Receive state from arm bridge (non-blocking)
     * @return true if new state received
     */
    bool ReceiveBridgeState();

    /**
     * @brief Check if bridge state is fresh
     * @param max_age_ms Maximum age in milliseconds
     * @return true if state is fresh
     */
    bool IsBridgeStateFresh(double max_age_ms = 250.0) const;

    /**
     * @brief Check if bridge state is valid
     */
    bool IsBridgeStateValid() const;

    /**
     * @brief Get arm bridge state
     */
    bool GetBridgeState(std::vector<float>* q, std::vector<float>* dq, std::vector<float>* tau) const;

    // ========================================================================
    // State Access (thread-safe)
    // ========================================================================

    /**
     * @brief Capture current state snapshot
     */
    ControllerStateSnapshot CaptureSnapshot() const;

    /**
     * @brief Restore from snapshot
     */
    void RestoreFromSnapshot(const ControllerStateSnapshot& snapshot);

    /**
     * @brief Get current hold position
     */
    std::vector<float> GetHoldPosition() const;

    /**
     * @brief Get arm joint count
     */
    int GetArmJointCount() const { return config_.arm_joint_count; }

    /**
     * @brief Get arm joint start index
     */
    int GetArmJointStartIndex() const { return config_.arm_joint_start_index; }

    /**
     * @brief Check if split control is enabled
     */
    bool IsSplitControlEnabled() const { return config_.arm_split_control_enabled; }

    /**
     * @brief Check if arm requires bridge state
     */
    bool RequiresBridgeState() const { return config_.arm_bridge_require_state; }

    // ========================================================================
    // Safety Limiting
    // ========================================================================

    /**
     * @brief Clip arm pose target to limits
     * @param target Target to clip (modified in place)
     * @param fallback Fallback if clipping fails
     * @param context Description for logging
     * @return true if successful or fallback used
     */
    bool ClipArmPoseTarget(std::vector<float>& target,
                          const std::vector<float>& fallback,
                          const char* context) const;

    /**
     * @brief Clip bridge command to limits
     * @param cmd Command to clip (modified in place)
     * @param fallback Fallback position
     * @param context Description for logging
     * @return true if successful or fallback used
     */
    bool ClipBridgeCommand(ArmBridgeCommand& cmd,
                          const std::vector<float>& fallback,
                          const char* context) const;

    // ========================================================================
    // IPC Interface
    // ========================================================================

    /**
     * @brief Poll arm command IPC socket
     */
    void PollCommandIpc();

    /**
     * @brief Poll arm bridge state IPC socket
     */
    void PollBridgeStateIpc();

    /**
     * @brief Set IPC socket file descriptors (external management)
     */
    void SetCommandIpcSocket(int fd);
    void SetBridgeIpcSockets(int cmd_fd, int state_fd);

    // ========================================================================
    // External State Integration
    // ========================================================================

    /**
     * @brief Update external state for arm feedback
     * @param q Joint positions
     * @param dq Joint velocities
     */
    void UpdateShadowState(const std::vector<float>& q, const std::vector<float>& dq);

    /**
     * @brief Read arm state into robot state (for observation)
     * @param state_q Output joint positions
     * @param state_dq Output joint velocities
     * @param state_tau Output joint torques
     * @return true if state is valid
     */
    bool ReadArmState(std::vector<float>* state_q,
                     std::vector<float>* state_dq,
                     std::vector<float>* state_tau);

    // ========================================================================
    // Transport Access
    // ========================================================================

    Go2X5ArmTransport::Transport* GetTransport() { return transport_.get(); }
    const Go2X5ArmTransport::Transport* GetTransport() const { return transport_.get(); }

    // ========================================================================
    // State Manager Access
    // ========================================================================

    Go2X5State::StateManager& GetStateManager() { return *state_manager_; }
    const Go2X5State::StateManager& GetStateManager() const { return *state_manager_; }

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get transport statistics
     */
    Go2X5ArmTransport::TransportStats GetTransportStats() const;

    /**
     * @brief Reset transport statistics
     */
    void ResetTransportStats();

private:
    bool initialized_ = false;
    ControllerConfig config_;
    Go2X5State::StateManager* state_manager_ = nullptr;
    std::unique_ptr<Go2X5ArmTransport::Transport> transport_;

    // IPC sockets (externally managed)
    int command_ipc_fd_ = -1;
    int bridge_cmd_fd_ = -1;
    int bridge_state_fd_ = -1;

    // Safety context
    mutable Go2X5SafetyGuard::ArmSafetyContext safety_context_;

    void UpdateSafetyContext();
};

} // namespace Go2X5ArmController

#endif // GO2_X5_ARM_CONTROLLER_HPP
