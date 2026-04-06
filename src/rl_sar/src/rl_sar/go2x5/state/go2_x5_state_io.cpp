
#include "rl_real_go2_x5.hpp"
#include "rl_sar/adapters/arx_adapter.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/ipc.hpp"
#include <mutex>
#include <chrono>

#if defined(__linux__)
#include <sys/socket.h>
#include <unistd.h>
#endif

// ============================================================================
// Arm State I/O Functions
// ============================================================================

void RL_Real_Go2X5::ReadArmStateFromExternal(RobotState<float> *state)
{
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }
    if (this->SyncArmStateFromAdapter(state))
    {
        return;
    }
    if (this->UseArmBridgeIpc())
    {
        this->PollArmBridgeIpcState();
    }

    std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
    auto bridge_state = this->CaptureArmBridgeRuntimeStateLocked();
    const auto decision = Go2X5ArmBridgeRuntime::EvaluateReadDecision(bridge_state);
    Go2X5ArmBridgeRuntime::ApplyReadDecision(&bridge_state, decision);
    this->ApplyArmBridgeRuntimeStateLocked(bridge_state);

    if (decision.warn_shadow_only)
    {
        if (this->arm_bridge_shadow_feedback_enabled)
        {
            std::cout << LOGGER::WARNING
                      << "Arm bridge topic is alive but reports shadow-only state. "
                      << "Treating arm feedback as invalid until a real backend state arrives."
                      << std::endl;
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Arm bridge topic is alive but reports shadow-only state. "
                      << "Ignoring shadow arm feedback until a real backend state arrives."
                      << std::endl;
        }
    }
    else if (decision.warn_timeout)
    {
        if (this->arm_bridge_shadow_feedback_enabled)
        {
            std::cout << LOGGER::WARNING
                      << "Arm bridge state is missing or stale. Using shadow arm state until bridge recovers."
                      << std::endl;
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Arm bridge state is missing or stale. Ignoring shadow arm feedback until bridge recovers."
                      << std::endl;
        }
    }

    for (int i = 0; i < this->arm_joint_count; ++i)
    {
        const int idx = this->arm_joint_start_index + i;
        if (idx < 0 || idx >= static_cast<int>(state->motor_state.q.size()))
        {
            continue;
        }

        if (decision.mode == Go2X5ArmBridgeRuntime::FeedbackMode::LiveState)
        {
            state->motor_state.q[idx] = this->arm_external_state_q[static_cast<size_t>(i)];
            state->motor_state.dq[idx] = this->arm_external_state_dq[static_cast<size_t>(i)];
            state->motor_state.tau_est[idx] = this->arm_external_state_tau[static_cast<size_t>(i)];
        }
        else if (decision.mode == Go2X5ArmBridgeRuntime::FeedbackMode::ShadowState &&
                 this->arm_bridge_shadow_feedback_enabled)
        {
            state->motor_state.q[idx] = this->arm_external_shadow_q[static_cast<size_t>(i)];
            state->motor_state.dq[idx] = this->arm_external_shadow_dq[static_cast<size_t>(i)];
            state->motor_state.tau_est[idx] = 0.0f;
        }
    }
}

void RL_Real_Go2X5::WriteArmCommandToExternal(const RobotCommand<float> *command)
{
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }

    std::vector<float> arm_q(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_dq(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_kp(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_kd(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_tau(static_cast<size_t>(this->arm_joint_count), 0.0f);
    const auto fixed_kp = GetFixedKp();
    const auto fixed_kd = GetFixedKd();
    for (int i = 0; i < this->arm_joint_count; ++i)
    {
        const int idx = this->arm_joint_start_index + i;
        if (idx < 0 || idx >= static_cast<int>(command->motor_command.q.size()))
        {
            continue;
        }
        arm_q[static_cast<size_t>(i)] = command->motor_command.q[idx];
        arm_dq[static_cast<size_t>(i)] = command->motor_command.dq[idx];
        arm_kp[static_cast<size_t>(i)] = command->motor_command.kp[idx];
        arm_kd[static_cast<size_t>(i)] = command->motor_command.kd[idx];
        arm_tau[static_cast<size_t>(i)] = command->motor_command.tau[idx];
    }

    std::vector<float> arm_hold_local;
    bool arm_hold_enabled_local = false;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_hold_enabled_local = this->arm_hold_enabled;
        arm_hold_local = this->arm_hold_position;
    }
    const bool in_rl_locomotion = this->IsInRLLocomotionState();
    const bool allow_passthrough = in_rl_locomotion || this->arm_safe_shutdown_active.load();
    if (allow_passthrough)
    {
        this->arm_non_rl_guard_warned = false;
    }
    else if (!this->arm_non_rl_guard_warned)
    {
        this->arm_non_rl_guard_warned = true;
        std::cout << LOGGER::WARNING
                  << "Arm passthrough blocked outside RL locomotion. Holding arm position until RL or shutdown sequence."
                  << std::endl;
    }

    const bool force_hold = !allow_passthrough;
    const bool use_hold_command = force_hold || arm_hold_enabled_local;
    if (force_hold && arm_hold_local.size() != static_cast<size_t>(this->arm_joint_count))
    {
        std::cout << LOGGER::WARNING
                  << "Arm passthrough blocked outside RL, but no valid arm hold pose is available."
                  << std::endl;
        return;
    }
    if (use_hold_command && arm_hold_local.size() == static_cast<size_t>(this->arm_joint_count))
    {
        for (int i = 0; i < this->arm_joint_count; ++i)
        {
            const int idx = this->arm_joint_start_index + i;
            arm_q[static_cast<size_t>(i)] = arm_hold_local[static_cast<size_t>(i)];
            arm_dq[static_cast<size_t>(i)] = 0.0f;
            arm_tau[static_cast<size_t>(i)] = 0.0f;
            if (idx >= 0 && idx < static_cast<int>(fixed_kp.size()))
            {
                arm_kp[static_cast<size_t>(i)] = fixed_kp[static_cast<size_t>(idx)];
            }
            if (idx >= 0 && idx < static_cast<int>(fixed_kd.size()))
            {
                arm_kd[static_cast<size_t>(i)] = fixed_kd[static_cast<size_t>(idx)];
            }
        }
    }

    if (!this->ClipArmBridgeCommandInPlace(
            arm_q, arm_dq, arm_kp, arm_kd, arm_tau, arm_hold_local, "Arm bridge command"))
    {
        if (arm_hold_local.size() == static_cast<size_t>(this->arm_joint_count))
        {
            for (int i = 0; i < this->arm_joint_count; ++i)
            {
                const int idx = this->arm_joint_start_index + i;
                arm_q[static_cast<size_t>(i)] = arm_hold_local[static_cast<size_t>(i)];
                arm_dq[static_cast<size_t>(i)] = 0.0f;
                arm_tau[static_cast<size_t>(i)] = 0.0f;
                arm_kp[static_cast<size_t>(i)] =
                    (idx >= 0 && idx < static_cast<int>(fixed_kp.size())) ? fixed_kp[static_cast<size_t>(idx)] : 0.0f;
                arm_kd[static_cast<size_t>(i)] =
                    (idx >= 0 && idx < static_cast<int>(fixed_kd.size())) ? fixed_kd[static_cast<size_t>(idx)] : 0.0f;
            }
            this->ClipArmBridgeCommandInPlace(
                arm_q, arm_dq, arm_kp, arm_kd, arm_tau, arm_hold_local, "Arm hold fallback");
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Arm bridge command rejected and no valid hold fallback is available."
                      << std::endl;
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        auto bridge_state = this->CaptureArmBridgeRuntimeStateLocked();
        Go2X5ArmBridgeRuntime::UpdateShadowState(&bridge_state, arm_q, arm_dq);
        this->ApplyArmBridgeRuntimeStateLocked(bridge_state);
    }

    rl_sar::protocol::ArmCommandFrame frame;
    frame.header.msg_type = rl_sar::protocol::FrameType::ArmCommand;
    frame.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
    frame.joint_count = static_cast<uint16_t>(this->arm_joint_count);
    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    frame.header.source_monotonic_ns = now_ns;
    frame.header.publish_monotonic_ns = now_ns;
    frame.command_expire_ns = now_ns + 15'000'000ULL;
    for (int i = 0; i < this->arm_joint_count && i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        frame.q[idx] = arm_q[idx];
        frame.dq[idx] = arm_dq[idx];
        frame.kp[idx] = arm_kp[idx];
        frame.kd[idx] = arm_kd[idx];
        frame.tau[idx] = arm_tau[idx];
    }

    if (this->arx_adapter_active_ && this->arx_adapter_)
    {
        this->arx_adapter_->SetCommand(frame);
        return;
    }

    this->WriteArmCommandFrameToExternal(frame);
}

void RL_Real_Go2X5::WriteArmCommandFrameToExternal(const rl_sar::protocol::ArmCommandFrame& input_frame)
{
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }

    std::vector<float> arm_q(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_dq(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_kp(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_kd(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_tau(static_cast<size_t>(this->arm_joint_count), 0.0f);
    std::vector<float> arm_hold_local;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_hold_local = this->arm_hold_position;
    }

    for (int i = 0; i < this->arm_joint_count && i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        arm_q[idx] = input_frame.q[idx];
        arm_dq[idx] = input_frame.dq[idx];
        arm_kp[idx] = input_frame.kp[idx];
        arm_kd[idx] = input_frame.kd[idx];
        arm_tau[idx] = input_frame.tau[idx];
    }

    if (!this->ClipArmBridgeCommandInPlace(
            arm_q, arm_dq, arm_kp, arm_kd, arm_tau, arm_hold_local, "Arm bridge command frame"))
    {
        if (arm_hold_local.size() == static_cast<size_t>(this->arm_joint_count))
        {
            const auto fixed_kp = GetFixedKp();
            const auto fixed_kd = GetFixedKd();
            for (int i = 0; i < this->arm_joint_count; ++i)
            {
                const size_t idx = static_cast<size_t>(i);
                const int joint_idx = this->arm_joint_start_index + i;
                arm_q[idx] = arm_hold_local[idx];
                arm_dq[idx] = 0.0f;
                arm_tau[idx] = 0.0f;
                arm_kp[idx] =
                    (joint_idx >= 0 && joint_idx < static_cast<int>(fixed_kp.size()))
                        ? fixed_kp[static_cast<size_t>(joint_idx)]
                        : 0.0f;
                arm_kd[idx] =
                    (joint_idx >= 0 && joint_idx < static_cast<int>(fixed_kd.size()))
                        ? fixed_kd[static_cast<size_t>(joint_idx)]
                        : 0.0f;
            }
            this->ClipArmBridgeCommandInPlace(
                arm_q, arm_dq, arm_kp, arm_kd, arm_tau, arm_hold_local, "Arm bridge hold fallback");
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Arm command frame rejected and no valid hold fallback is available."
                      << std::endl;
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        auto bridge_state = this->CaptureArmBridgeRuntimeStateLocked();
        Go2X5ArmBridgeRuntime::UpdateShadowState(&bridge_state, arm_q, arm_dq);
        this->ApplyArmBridgeRuntimeStateLocked(bridge_state);
    }

    rl_sar::protocol::ArmCommandFrame frame = input_frame;
    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    frame.header.seq = ++this->arm_bridge_command_seq_;
    frame.header.source_monotonic_ns = now_ns;
    frame.header.publish_monotonic_ns = now_ns;
    frame.header.validity_flags |= rl_sar::protocol::kValidityPayloadValid;
    frame.joint_count = static_cast<uint16_t>(this->arm_joint_count);
    if (frame.command_expire_ns == 0 || frame.command_expire_ns <= now_ns)
    {
        frame.command_expire_ns = now_ns + 15'000'000ULL;
    }
    for (int i = 0; i < this->arm_joint_count && i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        frame.q[idx] = arm_q[idx];
        frame.dq[idx] = arm_dq[idx];
        frame.kp[idx] = arm_kp[idx];
        frame.kd[idx] = arm_kd[idx];
        frame.tau[idx] = arm_tau[idx];
    }

    if (this->arx_adapter_active_ && this->arx_adapter_)
    {
        this->arx_adapter_->SetCommand(frame);
        return;
    }

    if (this->UseArmBridgeIpc())
    {
#if defined(__linux__)
        if (this->arm_bridge_cmd_socket_fd >= 0)
        {
            const auto bytes = rl_sar::protocol::SerializeArmCommandFrame(frame);
            const ssize_t sent = send(
                this->arm_bridge_cmd_socket_fd,
                bytes.data(),
                bytes.size(),
                0);
            if (sent < 0)
            {
                const auto now = std::chrono::steady_clock::now();
                const bool should_log =
                    this->arm_bridge_ipc_send_warn_stamp.time_since_epoch().count() == 0 ||
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - this->arm_bridge_ipc_send_warn_stamp).count() >= 1000;
                if (should_log)
                {
                    this->arm_bridge_ipc_send_warn_stamp = now;
                    std::cout << LOGGER::WARNING << "Arm bridge IPC send failed: "
                              << std::strerror(errno) << std::endl;
                }
            }
        }
#endif
        return;
    }

#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (!this->arm_bridge_cmd_topic.empty())
    {
#if defined(USE_ROS1) && defined(USE_ROS)
        if (this->arm_bridge_cmd_publisher)
        {
            std_msgs::Float32MultiArray msg;
            msg.data.reserve(static_cast<size_t>(this->arm_joint_count) * 5);
            msg.data.insert(msg.data.end(), arm_q.begin(), arm_q.end());
            msg.data.insert(msg.data.end(), arm_dq.begin(), arm_dq.end());
            msg.data.insert(msg.data.end(), arm_kp.begin(), arm_kp.end());
            msg.data.insert(msg.data.end(), arm_kd.begin(), arm_kd.end());
            msg.data.insert(msg.data.end(), arm_tau.begin(), arm_tau.end());
            this->arm_bridge_cmd_publisher.publish(msg);
        }
#elif defined(USE_ROS2) && defined(USE_ROS)
        if (this->arm_bridge_cmd_publisher)
        {
            std_msgs::msg::Float32MultiArray msg;
            msg.data.reserve(static_cast<size_t>(this->arm_joint_count) * 5);
            msg.data.insert(msg.data.end(), arm_q.begin(), arm_q.end());
            msg.data.insert(msg.data.end(), arm_dq.begin(), arm_dq.end());
            msg.data.insert(msg.data.end(), arm_kp.begin(), arm_kp.end());
            msg.data.insert(msg.data.end(), arm_kd.begin(), arm_kd.end());
            msg.data.insert(msg.data.end(), arm_tau.begin(), arm_tau.end());
            this->arm_bridge_cmd_publisher->publish(msg);
        }
#endif
    }
#endif
}

void RL_Real_Go2X5::ApplyArmHold(const std::vector<float>& target, const char* reason)
{
    if (this->arm_command_size <= 0)
    {
        return;
    }

    if (target.size() != static_cast<size_t>(this->arm_command_size))
    {
        std::cout << LOGGER::WARNING << "Ignore arm hold update: target size mismatch" << std::endl;
        return;
    }

    std::vector<float> target_local = target;
    std::vector<float> fallback_local;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        fallback_local = this->arm_hold_position;
    }
    if (!this->ClipArmPoseTargetInPlace(target_local, fallback_local, reason))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    auto state = this->CaptureArmCommandStateLocked();
    Go2X5ArmRuntime::ApplyHoldTarget(&state, target_local);
    this->ApplyArmCommandStateLocked(state);

    const float smoothing_time = config_->GetArmCommandSmoothingTime();
    std::cout << LOGGER::INFO << reason << " (smooth=" << smoothing_time << "s)" << std::endl;
}

bool RL_Real_Go2X5::ArmCommandDifferent(const std::vector<float>& a, const std::vector<float>& b) const
{
    return Go2X5ArmRuntime::CommandDifferent(a, b);
}
