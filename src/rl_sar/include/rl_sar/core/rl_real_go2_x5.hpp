#ifndef RL_REAL_GO2_X5_HPP
#define RL_REAL_GO2_X5_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"
#include "rl_sar/go2x5/state/go2_x5_state_manager.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_transport.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_controller.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_runtime.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_bridge_runtime.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/comm/go2_x5_ipc_protocol.hpp"
#include "rl_sar/protocol/go2_x5_protocol.hpp"
#include "rl_sar/runtime/supervisor/go2_x5_supervisor.hpp"
#include "library/core/config/deploy_manifest_runtime.hpp"
#include "fsm_go2_x5.hpp"
#include "library/core/config/config_loader.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

class RL_Real_Go2X5 : public RL
{
public:
    RL_Real_Go2X5(int argc, char **argv);
    ~RL_Real_Go2X5();
    void SafeShutdownNow();
    bool LoopExceptionRequested() const;

#if defined(USE_ROS2) && defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    void InitLowCmd();
    int QueryMotionStatus();
    std::string QueryServiceName(std::string form, std::string name);
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void LowStateMessageHandler(const void *messages);
    void JoystickHandler(const void *message);
    MotionSwitcherClient msc;
    unitree_go::msg::dds_::LowCmd_ unitree_low_command{};
    std::array<float, 4> unitree_imu_quaternion{{1.0f, 0.0f, 0.0f, 0.0f}};
    std::array<float, 3> unitree_imu_gyroscope{{0.0f, 0.0f, 0.0f}};
    std::array<float, 20> unitree_motor_q{};
    std::array<float, 20> unitree_motor_dq{};
    std::array<float, 20> unitree_motor_tau{};
    float unitree_joy_lx = 0.0f;
    float unitree_joy_ly = 0.0f;
    float unitree_joy_rx = 0.0f;
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;
    xKeySwitchUnion unitree_joy;

    std::unique_ptr<RLConfig::ConfigLoader> config_loader_;
    std::unique_ptr<RLConfig::DeployManifestRuntime> deploy_manifest_runtime_;
    std::unique_ptr<Go2X5Config::Go2X5Config> config_;
    std::unique_ptr<Go2X5State::StateManager> state_manager_;
    std::unique_ptr<Go2X5ArmController::ArmController> arm_controller_;
    std::unique_ptr<Go2X5Supervisor::Supervisor> supervisor_;

    std::string deploy_manifest_path_ = RLConfig::DeployManifestRuntime::kDefaultManifestPath;
    bool manifest_valid_ = false;
    bool runtime_config_valid_ = false;
    bool runtime_ros2_enabled_explicit_ = false;
    bool runtime_arm_bridge_transport_explicit_ = false;
    std::string manifest_hash_;
    std::chrono::steady_clock::time_point body_state_stamp_{};
    uint64_t body_state_seq_ = 0;
    uint64_t arm_state_seq_ = 0;
    uint64_t policy_seq_ = 0;
    bool body_state_seen_ = false;
    bool arm_state_seen_ = false;
    bool body_state_seq_pending_ = false;
    bool arm_state_seq_pending_ = false;
    bool policy_seen_ = false;

    void InitializeConfigLoader();
    void InitializeRuntimeOptions(int argc, char **argv);
    void InitializeArmConfig();
    void InitializeArmCommandState();
    void InitializeArmChannelConfig();
    void InitializeRealDeploySafetyConfig();
    void InitializeSupervisor();
    void ValidateJointMappingOrThrow(const char* stage) const;
    Go2X5Supervisor::WatchdogInput BuildSupervisorInput() const;
    void RefreshSupervisorState(const char* source);

    int GetNumDofs() const;
    float GetDt() const;
    int GetDecimation() const;
    std::vector<float> GetDefaultDofPos() const;
    std::vector<float> GetFixedKp() const;
    std::vector<float> GetFixedKd() const;
    std::vector<int> GetJointMapping() const;

    Go2X5ArmRuntime::CommandState CaptureArmCommandStateLocked() const;
    void ApplyArmCommandStateLocked(const Go2X5ArmRuntime::CommandState& state);
    Go2X5ArmBridgeRuntime::RuntimeState CaptureArmBridgeRuntimeStateLocked() const;
    void ApplyArmBridgeRuntimeStateLocked(const Go2X5ArmBridgeRuntime::RuntimeState& state);
    Go2X5ControlLogic::ArmRuntimeStateSnapshot CaptureArmRuntimeStateLocked() const;
    void RestoreArmRuntimeStateLocked(const Go2X5ControlLogic::ArmRuntimeStateSnapshot& snapshot);
    void RestoreArmRuntimeStateLocked(const Go2X5ArmRuntime::CommandState& cmd,
                                      const Go2X5ArmBridgeRuntime::RuntimeState& bridge);

    void SetupArmCommandSubscriber();
    void SetupArmCommandIpc();
    void CloseArmCommandIpc();
    void PollArmCommandIpc();
    void SetupArmBridgeInterface();
    void SetupArmBridgeIpc();
    void CloseArmBridgeIpc();
    void PollArmBridgeIpcState();
    bool IsArmBridgeStateFreshLocked() const;
    bool IsArmJointIndex(int idx) const;
    bool IsInRLLocomotionState() const;
    bool UseExclusiveRealDeployControl() const;
    bool UseArmBridgeIpc() const;
    std::vector<float> GetDefaultWholeBodyLowerLimits() const;
    std::vector<float> GetDefaultWholeBodyUpperLimits() const;
    std::vector<float> GetDefaultWholeBodyVelocityLimits() const;
    std::vector<float> GetDefaultWholeBodyEffortLimits() const;
    std::vector<float> GetDefaultWholeBodyKpLimits() const;
    std::vector<float> GetDefaultWholeBodyKdLimits() const;
    std::vector<float> GetDefaultArmLowerLimits() const;
    std::vector<float> GetDefaultArmUpperLimits() const;
    bool ClipWholeBodyCommand(RobotCommand<float> *command, const char* context) const;
    bool ClipArmPoseTargetInPlace(std::vector<float>& target,
                                  const std::vector<float>& fallback,
                                  const char* context) const;
    bool ClipArmBridgeCommandInPlace(std::vector<float>& q,
                                     std::vector<float>& dq,
                                     std::vector<float>& kp,
                                     std::vector<float>& kd,
                                     std::vector<float>& tau,
                                     const std::vector<float>& q_fallback,
                                     const char* context) const;
    bool ValidateArmPoseTarget(const std::vector<float>& target, const char* context) const;
    bool ValidateArmBridgeStateSample(const std::vector<float>& q,
                                      const std::vector<float>& dq,
                                      const std::vector<float>& tau,
                                      const char* context) const;
    void HandleArmJointCommandData(const std::vector<float>& data, const char* context);
    void HandleArmBridgeStateData(const std::vector<float>& data,
                                  bool state_from_backend,
                                  const char* context,
                                  bool has_transport_seq = false,
                                  uint64_t transport_seq = 0);
    void HandleArmJointCommandFrame(const rl_sar::protocol::ArmCommandFrame& frame,
                                    const char* context);
    void HandleArmBridgeStateFrame(const rl_sar::protocol::ArmStateFrame& frame,
                                  const char* context);
    void ReadArmStateFromExternal(RobotState<float> *state);
    void WriteArmCommandToExternal(const RobotCommand<float> *command);
    void ApplyArmHold(const std::vector<float>& target, const char* reason);
    bool ArmCommandDifferent(const std::vector<float>& a, const std::vector<float>& b) const;
    void ExecuteSafeShutdownSequence();
    std::vector<float> BuildSafeShutdownTargetPose(const std::vector<float>& default_pos) const;
    void PublishWholeBodyPose(const std::vector<float>& pose,
                              const std::vector<float>& kp,
                              const std::vector<float>& kd);
    void MaybePublishKey1CmdVel();
    void RecordPolicyInferenceTick();
    void HandleLoopException(const std::string& loop_name, const std::string& error);
    void HandleKey2ArmHold();
    void HandleKey3ArmDefault();
    void HandleKey4ArmHoldToggle();

    float cmd_vel_alpha = 0.2f;
    bool cmd_vel_has_filtered = false;
    bool key1_navigation_cmd_published = false;

    int arm_command_size = 0;
    bool arm_hold_enabled = true;
    bool arm_runtime_params_ready = false;
    bool enable_ros2_runtime = true;
    std::string arm_joint_command_topic = "/arm_joint_pos_cmd";
    std::string arm_joint_command_topic_active;
    int arm_joint_command_port = Go2X5IPC::kDefaultJointCommandPort;
    int arm_joint_command_socket_fd = -1;
    std::vector<float> arm_joint_command_latest;
    std::vector<float> arm_topic_command_latest;
    bool arm_topic_command_received = false;
    std::vector<float> arm_hold_position;
    std::vector<float> arm_command_smoothing_start;
    std::vector<float> arm_command_smoothing_target;
    std::vector<float> arm_command_smoothed;
    bool arm_command_initialized = false;
    int arm_command_smoothing_ticks = 0;
    int arm_command_smoothing_counter = 0;
    std::string arm_control_mode = "unitree";
    int arm_joint_start_index = 12;
    int arm_joint_count = 6;
    bool arm_split_control_enabled = false;
    bool arm_bridge_state_valid = false;
    bool arm_bridge_require_state = true;
    bool arm_bridge_require_live_state = true;
    bool arm_bridge_shadow_feedback_enabled = true;
    bool arm_bridge_state_timeout_warned = false;
    bool arm_bridge_shadow_mode_warned = false;
    bool arm_bridge_state_from_backend = false;
    float arm_bridge_state_timeout_sec = 0.25f;
    std::string arm_bridge_transport = "ros";
    std::string arm_bridge_ipc_host = Go2X5IPC::kDefaultHost;
    int arm_bridge_cmd_port = Go2X5IPC::kDefaultCommandPort;
    int arm_bridge_state_port = Go2X5IPC::kDefaultStatePort;
    int arm_bridge_cmd_socket_fd = -1;
    int arm_bridge_state_socket_fd = -1;
    std::string arm_bridge_cmd_topic = "/arx_x5/joint_cmd";
    std::string arm_bridge_state_topic = "/arx_x5/joint_state";
    std::vector<float> arm_external_state_q;
    std::vector<float> arm_external_state_dq;
    std::vector<float> arm_external_state_tau;
    std::vector<float> arm_external_shadow_q;
    std::vector<float> arm_external_shadow_dq;
    std::chrono::steady_clock::time_point arm_bridge_state_stamp;
    std::chrono::steady_clock::time_point arm_tracking_error_high_stamp;
    uint64_t arm_bridge_command_seq_ = 0;
    bool arm_bridge_state_stream_logged = false;
    bool arm_non_rl_guard_warned = false;
    bool arm_tracking_error_high_runtime_ = false;
    double arm_tracking_error_limit_ = 0.0;
    float joystick_deadband = 0.05f;
    bool safe_shutdown_done = false;
    std::atomic<bool> arm_safe_shutdown_active{false};
    bool real_deploy_exclusive_keyboard_control = true;
    bool cmd_vel_input_ignored_warned = false;
    bool policy_inference_log_enabled = true;
    float last_policy_inference_hz = 0.0f;
    std::chrono::steady_clock::time_point last_policy_inference_stamp{};
    std::atomic<bool> operator_estop_requested_{false};
    std::atomic<bool> operator_fault_reset_requested_{false};
    std::atomic<bool> operator_manual_arm_request_{false};
    std::atomic<bool> policy_health_bad_runtime_{false};
    std::atomic<bool> body_dds_write_ok_runtime_{true};
    mutable std::chrono::steady_clock::time_point whole_body_clip_warn_stamp{};
    std::chrono::steady_clock::time_point arm_bridge_ipc_send_warn_stamp{};
    std::atomic<bool> loop_exception_requested{false};
    std::mutex safe_shutdown_mutex;
    std::mutex loop_exception_mutex;
    std::string loop_exception_message;
    std::vector<float> whole_body_joint_lower_limits;
    std::vector<float> whole_body_joint_upper_limits;
    std::vector<float> whole_body_velocity_limits;
    std::vector<float> whole_body_effort_limits;
    std::vector<float> whole_body_kp_limits;
    std::vector<float> whole_body_kd_limits;
    std::vector<float> arm_joint_lower_limits;
    std::vector<float> arm_joint_upper_limits;

    std::mutex cmd_vel_mutex;
    std::mutex arm_command_mutex;
    std::mutex arm_external_state_mutex;
    std::mutex unitree_state_mutex;
    mutable std::mutex supervisor_mutex;

#if defined(USE_ROS1) && defined(USE_ROS)
    std::shared_ptr<ros::NodeHandle> ros1_nh;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist cmd_vel_filtered;
    ros::Publisher cmd_vel_publisher;
    ros::Subscriber cmd_vel_subscriber;
    ros::Subscriber arm_joint_command_subscriber;
    ros::Publisher arm_bridge_cmd_publisher;
    ros::Subscriber arm_bridge_state_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void ArmJointCommandCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void ArmBridgeStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Twist cmd_vel_filtered;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_joint_command_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_bridge_cmd_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_bridge_state_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void ArmJointCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void ArmBridgeStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
#endif
};

#endif // RL_REAL_GO2_X5_HPP
