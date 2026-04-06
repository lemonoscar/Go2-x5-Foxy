
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/control_logic.hpp"
#include "rl_sar/go2x5/ipc.hpp"
#include <chrono>
#include <iomanip>

// ============================================================================
// Utility Functions
// ============================================================================

bool RL_Real_Go2X5::UseExclusiveRealDeployControl() const
{
    return this->real_deploy_exclusive_keyboard_control;
}

bool RL_Real_Go2X5::UseArmBridgeIpc() const
{
    return Go2X5IPC::IsIpcTransport(this->arm_bridge_transport);
}

bool RL_Real_Go2X5::IsArmJointIndex(int idx) const
{
    return idx >= this->arm_joint_start_index &&
           idx < (this->arm_joint_start_index + this->arm_joint_count);
}

bool RL_Real_Go2X5::IsInRLLocomotionState() const
{
    return this->fsm.current_state_ &&
           this->fsm.current_state_->GetStateName().find("RLLocomotion") != std::string::npos;
}

void RL_Real_Go2X5::HandleLoopException(const std::string& loop_name, const std::string& error)
{
    bool expected = false;
    if (!this->loop_exception_requested.compare_exchange_strong(expected, true))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(this->loop_exception_mutex);
        this->loop_exception_message = loop_name + ": " + error;
    }
    std::cout << LOGGER::ERROR << "Loop exception captured, requesting safe shutdown: "
              << this->loop_exception_message << std::endl;
}

bool RL_Real_Go2X5::LoopExceptionRequested() const
{
    return this->loop_exception_requested.load();
}

void RL_Real_Go2X5::MaybePublishKey1CmdVel()
{
#if !defined(USE_CMAKE) && defined(USE_ROS)
    const bool key1_pressed = this->control.current_keyboard == Input::Keyboard::Num1;
    if (Go2X5ControlLogic::ShouldResetKey1NavigationPublished(this->control.navigation_mode))
    {
        this->key1_navigation_cmd_published = false;
        return;
    }

    Go2X5ControlLogic::Key1NavigationPublishRequest request;
    request.exclusive_control = this->UseExclusiveRealDeployControl();
    request.navigation_mode = this->control.navigation_mode;
    request.key1_pressed = key1_pressed;
    request.already_published = this->key1_navigation_cmd_published;
    request.publish_enabled = config_->GetKey1PublishCmdVelOnNavigation();
    if (!Go2X5ControlLogic::ShouldPublishKey1NavigationCmd(request))
    {
        return;
    }

    const auto navigation_cmd = Go2X5ControlLogic::BuildKey1NavigationCommand(
        config_->GetKey1NavigationCmdX(),
        config_->GetKey1NavigationCmdY(),
        config_->GetKey1NavigationCmdYaw());

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist msg;
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist msg;
#endif
    msg.linear.x = navigation_cmd.x;
    msg.linear.y = navigation_cmd.y;
    msg.linear.z = 0.0f;
    msg.angular.x = 0.0f;
    msg.angular.y = 0.0f;
    msg.angular.z = navigation_cmd.yaw;

    {
        std::lock_guard<std::mutex> lock(this->cmd_vel_mutex);
        this->cmd_vel = msg;
        this->cmd_vel_filtered = msg;
        this->cmd_vel_has_filtered = true;
    }
    this->control.x = static_cast<float>(msg.linear.x);
    this->control.y = static_cast<float>(msg.linear.y);
    this->control.yaw = static_cast<float>(msg.angular.z);
    this->key1_navigation_cmd_published = true;

#if defined(USE_ROS1) && defined(USE_ROS)
    if (this->cmd_vel_publisher)
    {
        this->cmd_vel_publisher.publish(msg);
    }
#elif defined(USE_ROS2) && defined(USE_ROS)
    if (this->cmd_vel_publisher)
    {
        this->cmd_vel_publisher->publish(msg);
    }
#endif

    std::cout << std::endl << LOGGER::INFO
              << "Key[1] pressed: publish /cmd_vel x=" << msg.linear.x
              << " y=" << msg.linear.y
              << " yaw=" << msg.angular.z << std::endl;
#endif
}

void RL_Real_Go2X5::RecordPolicyInferenceTick()
{
    const auto now = std::chrono::steady_clock::now();
    ++this->policy_seq_;
    this->policy_seen_ = true;
    if (this->last_policy_inference_stamp.time_since_epoch().count() > 0)
    {
        const double dt_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - this->last_policy_inference_stamp).count();
        if (dt_sec > 1e-6)
        {
            this->last_policy_inference_hz = static_cast<float>(1.0 / dt_sec);
        }
    }
    this->last_policy_inference_stamp = now;

    if (this->policy_inference_log_enabled && this->last_policy_inference_hz > 0.0f)
    {
        std::cout << "\r\033[K" << LOGGER::INFO << "Policy inference frequency: "
                  << std::fixed << std::setprecision(2) << this->last_policy_inference_hz
                  << " Hz" << std::flush;
    }

    this->RefreshSupervisorState("policy_inference");
}
