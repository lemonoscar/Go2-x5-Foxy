
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/state/go2_x5_state_manager.hpp"
#include <cmath>
#include <mutex>

namespace
{

uint64_t MonotonicNowNs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

}

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
                  << " values, got " << static_cast<size_t>(data.size()) << std::endl;
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

void RL_Real_Go2X5::HandleArmJointCommandFrame(
    const rl_sar::protocol::ArmCommandFrame& frame,
    const char* context)
{
    if (frame.joint_count != rl_sar::protocol::kArmJointCount)
    {
        std::cout << LOGGER::WARNING
                  << "Ignore " << context << ": expect joint_count="
                  << rl_sar::protocol::kArmJointCount
                  << ", got " << frame.joint_count << std::endl;
        return;
    }
    if (rl_sar::protocol::IsCommandExpired(MonotonicNowNs(), frame.command_expire_ns))
    {
        std::cout << LOGGER::WARNING << "Ignore " << context << ": arm command frame expired."
                  << std::endl;
        return;
    }

    this->HandleArmJointCommandData(
        std::vector<float>(frame.q.begin(), frame.q.end()),
        context);
}

void RL_Real_Go2X5::HandleArmBridgeStateData(
    const std::vector<float>& data,
    const bool state_from_backend,
    const char* context,
    const bool has_transport_seq,
    const uint64_t transport_seq)
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

    std::vector<float> target_pose;
    {
        std::lock_guard<std::mutex> command_lock(this->arm_command_mutex);
        target_pose = this->arm_joint_command_latest;
        if (target_pose.size() != sample.q.size())
        {
            target_pose = this->arm_hold_position;
        }
    }

    bool tracking_error_high = false;
    bool tracking_error_changed = false;
    Go2X5ArmBridgeRuntime::ApplyStateSampleResult result;
    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        auto bridge_state = this->CaptureArmBridgeRuntimeStateLocked();
        result = Go2X5ArmBridgeRuntime::ApplyStateSample(&bridge_state, sample, state_from_backend);
        this->ApplyArmBridgeRuntimeStateLocked(bridge_state);

        if (!sample.q.empty() &&
            target_pose.size() == sample.q.size() &&
            this->arm_tracking_error_limit_ > 0.0)
        {
            double squared_error = 0.0;
            for (size_t i = 0; i < sample.q.size(); ++i)
            {
                const double diff = static_cast<double>(sample.q[i]) - static_cast<double>(target_pose[i]);
                squared_error += diff * diff;
            }

            const double tracking_error_norm = std::sqrt(squared_error);
            const auto now = std::chrono::steady_clock::now();
            const uint64_t window_us = this->supervisor_
                ? this->supervisor_->config().arm_tracking_error_window_us
                : 200'000ULL;
            if (tracking_error_norm > this->arm_tracking_error_limit_)
            {
                if (this->arm_tracking_error_high_stamp.time_since_epoch().count() == 0)
                {
                    this->arm_tracking_error_high_stamp = now;
                }
                const auto age_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        now - this->arm_tracking_error_high_stamp).count());
                tracking_error_high = age_us >= window_us;
            }
            else
            {
                this->arm_tracking_error_high_stamp = std::chrono::steady_clock::time_point{};
            }
        }

        tracking_error_changed = (tracking_error_high != this->arm_tracking_error_high_runtime_);
        this->arm_tracking_error_high_runtime_ = tracking_error_high;

        this->arm_state_seen_ = true;
        if (has_transport_seq)
        {
            this->arm_state_seq_ = transport_seq;
            this->arm_state_seq_pending_ = false;
        }
        else
        {
            this->arm_state_seq_pending_ = true;
        }
        if (this->state_manager_)
        {
            this->state_manager_->SetArmBridgeConnected(this->arm_bridge_state_valid && this->arm_bridge_state_from_backend);
        }
    }

    if (result.stream_detected)
    {
        std::cout << LOGGER::INFO << "Arm bridge state stream detected: transport="
                  << (this->UseArmBridgeIpc() ? "ipc" : "ros")
                  << ", dof=" << static_cast<int>(this->arm_joint_count) << std::endl;
    }
    else if (result.warn_shadow_only)
    {
        std::cout << LOGGER::WARNING
                  << "Arm bridge feedback is shadow-only. Real arm feedback is still unavailable."
                  << std::endl;
    }
    if (tracking_error_changed)
    {
        std::cout << LOGGER::WARNING
                  << "[SupervisorInput] source=" << context
                  << " reason=arm_tracking_error_"
                  << (tracking_error_high ? "high" : "ok")
                  << std::endl;
    }

    this->RefreshSupervisorState(context);
}

void RL_Real_Go2X5::HandleArmBridgeStateFrame(
    const rl_sar::protocol::ArmStateFrame& frame,
    const char* context)
{
    if (frame.joint_count != rl_sar::protocol::kArmJointCount)
    {
        std::cout << LOGGER::WARNING
                  << "Ignore " << context << ": expect joint_count="
                  << rl_sar::protocol::kArmJointCount
                  << ", got " << frame.joint_count << std::endl;
        return;
    }

    std::vector<float> payload;
    payload.reserve(static_cast<size_t>(this->arm_joint_count) * 3);
    payload.insert(payload.end(), frame.q.begin(), frame.q.end());
    payload.insert(payload.end(), frame.dq.begin(), frame.dq.end());
    payload.insert(payload.end(), frame.tau.begin(), frame.tau.end());

    const bool state_from_backend =
        (frame.header.validity_flags & rl_sar::protocol::kValidityFromBackend) != 0u;
    this->HandleArmBridgeStateData(payload, state_from_backend, context, true, frame.header.seq);
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

#endif
