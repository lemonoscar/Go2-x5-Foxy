
#include "rl_real_go2_x5.hpp"
#include <mutex>

// ============================================================================
// ROS Callbacks and Data Handlers
// ============================================================================

void RL_Real_Go2X5::HandleArmJointCommandData(const std::vector<float>& data, const char* context)
{
    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    if (this->arm_command_size <= 0)
    {
        return;
    }
    if (data.size() < static_cast<size_t>(this->arm_command_size))
    {
        std::cout << LOGGER::WARNING
                  << "Ignore " << context << ": expect " << this->arm_command_size
                  << " values, got " << data.size() << std::endl;
        return;
    }

    std::vector<float> target(static_cast<size_t>(this->arm_command_size), 0.0f);
    const size_t count = static_cast<size_t>(this->arm_command_size);
    for (size_t i = 0; i < count; ++i)
    {
        target[i] = data[i];
    }
    if (!this->ClipArmPoseTargetInPlace(target, this->arm_hold_position, context))
    {
        return;
    }

    if (this->arm_joint_command_latest.size() != static_cast<size_t>(this->arm_command_size))
    {
        this->arm_joint_command_latest.assign(static_cast<size_t>(this->arm_command_size), 0.0f);
    }
    if (this->arm_topic_command_latest.size() != static_cast<size_t>(this->arm_command_size))
    {
        this->arm_topic_command_latest.assign(static_cast<size_t>(this->arm_command_size), 0.0f);
    }

    for (size_t i = 0; i < count; ++i)
    {
        this->arm_joint_command_latest[i] = target[i];
        this->arm_topic_command_latest[i] = target[i];
    }
    this->arm_topic_command_received = true;
}

void RL_Real_Go2X5::HandleArmBridgeStateData(
    const std::vector<float>& data,
    const bool state_from_backend,
    const char* context)
{
    if (this->arm_joint_count <= 0)
    {
        return;
    }

    Go2X5ArmBridgeRuntime::StateSample sample;
    if (!Go2X5ArmBridgeRuntime::ParseStatePayload(data, this->arm_joint_count, &sample))
    {
        return;
    }

    if (!this->ValidateArmBridgeStateSample(sample.q, sample.dq, sample.tau, context))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
    auto bridge_state = this->CaptureArmBridgeRuntimeStateLocked();
    const auto result = Go2X5ArmBridgeRuntime::ApplyStateSample(&bridge_state, sample, state_from_backend);
    this->ApplyArmBridgeRuntimeStateLocked(bridge_state);
    if (result.stream_detected)
    {
        std::cout << LOGGER::INFO << "Arm bridge state stream detected: transport="
                  << (this->UseArmBridgeIpc() ? "ipc" : "ros")
                  << ", dof=" << this->arm_joint_count << std::endl;
    }
    else if (result.warn_shadow_only)
    {
        std::cout << LOGGER::WARNING
                  << "Arm bridge feedback is shadow-only. Real arm feedback is still unavailable."
                  << std::endl;
    }
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real_Go2X5::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    if (this->UseExclusiveRealDeployControl())
    {
        if (!this->cmd_vel_input_ignored_warned)
        {
            this->cmd_vel_input_ignored_warned = true;
            std::cout << LOGGER::WARNING
                      << "Ignore /cmd_vel in exclusive real deploy control mode. Use keyboard 0/1/2 only."
                      << std::endl;
        }
        return;
    }

    std::lock_guard<std::mutex> lock(this->cmd_vel_mutex);
    this->cmd_vel = *msg;
    if (!this->cmd_vel_has_filtered)
    {
        this->cmd_vel_filtered = *msg;
        this->cmd_vel_has_filtered = true;
        return;
    }

    const float a = this->cmd_vel_alpha;
    this->cmd_vel_filtered.linear.x = a * msg->linear.x + (1.0f - a) * this->cmd_vel_filtered.linear.x;
    this->cmd_vel_filtered.linear.y = a * msg->linear.y + (1.0f - a) * this->cmd_vel_filtered.linear.y;
    this->cmd_vel_filtered.linear.z = a * msg->linear.z + (1.0f - a) * this->cmd_vel_filtered.linear.z;
    this->cmd_vel_filtered.angular.x = a * msg->angular.x + (1.0f - a) * this->cmd_vel_filtered.angular.x;
    this->cmd_vel_filtered.angular.y = a * msg->angular.y + (1.0f - a) * this->cmd_vel_filtered.angular.y;
    this->cmd_vel_filtered.angular.z = a * msg->angular.z + (1.0f - a) * this->cmd_vel_filtered.angular.z;
}

void RL_Real_Go2X5::ArmJointCommandCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const std_msgs::Float32MultiArray::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const std_msgs::msg::Float32MultiArray::SharedPtr msg
#endif
)
{
    if (!msg)
    {
        return;
    }
    this->HandleArmJointCommandData(msg->data, this->arm_joint_command_topic.c_str());
}

void RL_Real_Go2X5::ArmBridgeStateCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const std_msgs::Float32MultiArray::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const std_msgs::msg::Float32MultiArray::SharedPtr msg
#endif
)
{
    if (!msg)
    {
        return;
    }
    bool state_from_backend = false;
    const size_t n = static_cast<size_t>(std::max(0, this->arm_joint_count));
    std::vector<float> payload = msg->data;
    if (msg->data.size() == (3 * n + 1) || msg->data.size() == (n + 1))
    {
        state_from_backend = msg->data.back() > 0.5f;
        payload.pop_back();
    }
    this->HandleArmBridgeStateData(payload, state_from_backend, "Arm bridge state");
}
#endif
