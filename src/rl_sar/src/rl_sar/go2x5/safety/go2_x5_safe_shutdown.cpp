
#include "rl_sar/core/rl_real_go2_x5.hpp"
#include "rl_sar/adapters/unitree_adapter.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "loop.hpp"
#include <chrono>
#include <thread>
#include <cmath>

// ============================================================================
// Safe Shutdown Sequence
// ============================================================================

std::vector<float> RL_Real_Go2X5::BuildSafeShutdownTargetPose(const std::vector<float>& default_pos) const
{
    const int num_dofs = GetNumDofs();
    std::vector<float> target = config_->GetShutdownLiePose();
    if (target.size() != static_cast<size_t>(num_dofs))
    {
        target = default_pos;
    }
    if (target.size() != static_cast<size_t>(num_dofs))
    {
        target.assign(static_cast<size_t>(num_dofs), 0.0f);
    }

    static const std::vector<float> kLegLyingPose = {
        0.00f, 1.36f, -2.65f,
        0.00f, 1.36f, -2.65f,
        0.00f, 1.36f, -2.65f,
        0.00f, 1.36f, -2.65f
    };
    const int leg_dofs = std::min(12, num_dofs);
    for (int i = 0; i < leg_dofs && i < static_cast<int>(kLegLyingPose.size()); ++i)
    {
        target[static_cast<size_t>(i)] = kLegLyingPose[static_cast<size_t>(i)];
    }

    const int arm_size = std::max(0, std::min(this->arm_joint_count, num_dofs));
    const int arm_start = std::max(0, std::min(this->arm_joint_start_index, num_dofs));
    if (arm_size > 0 && arm_start + arm_size <= num_dofs)
    {
        std::vector<float> arm_target = config_->GetArmShutdownPose();
        if (arm_target.size() != static_cast<size_t>(arm_size))
        {
            arm_target = config_->GetArmHoldPose();
        }
        if (arm_target.size() != static_cast<size_t>(arm_size) &&
            default_pos.size() >= static_cast<size_t>(arm_start + arm_size))
        {
            arm_target.assign(
                default_pos.begin() + static_cast<long>(arm_start),
                default_pos.begin() + static_cast<long>(arm_start + arm_size));
        }
        if (arm_target.size() == static_cast<size_t>(arm_size))
        {
            for (int i = 0; i < arm_size; ++i)
            {
                target[static_cast<size_t>(arm_start + i)] = arm_target[static_cast<size_t>(i)];
            }
        }
    }
    return target;
}

void RL_Real_Go2X5::PublishWholeBodyPose(const std::vector<float>& pose,
    const std::vector<float>& kp, const std::vector<float>& kd)
{
    const int num_dofs = GetNumDofs();
    const auto joint_mapping = GetJointMapping();
    if (pose.size() < static_cast<size_t>(num_dofs) || joint_mapping.size() < static_cast<size_t>(num_dofs))
    {
        return;
    }

    RobotCommand<float> command_local;
    command_local.motor_command.resize(static_cast<size_t>(num_dofs));
    for (int i = 0; i < num_dofs; ++i)
    {
        command_local.motor_command.q[static_cast<size_t>(i)] = pose[static_cast<size_t>(i)];
        command_local.motor_command.dq[static_cast<size_t>(i)] = 0.0f;
        command_local.motor_command.tau[static_cast<size_t>(i)] = 0.0f;
        command_local.motor_command.kp[static_cast<size_t>(i)] =
            (i < static_cast<int>(kp.size())) ? kp[static_cast<size_t>(i)] : 40.0f;
        command_local.motor_command.kd[static_cast<size_t>(i)] =
            (i < static_cast<int>(kd.size())) ? kd[static_cast<size_t>(i)] : 3.0f;
    }

    this->ClipWholeBodyCommand(&command_local, "Whole-body pose publish");

    for (int i = 0; i < num_dofs; ++i)
    {
        const int mapped = joint_mapping[static_cast<size_t>(i)];
        if (mapped < 0 || mapped >= 20) continue;

        if (this->arm_split_control_enabled && this->IsArmJointIndex(i))
        {
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x00;
            this->unitree_low_command.motor_cmd()[mapped].q() = PosStopF;
            this->unitree_low_command.motor_cmd()[mapped].dq() = VelStopF;
            this->unitree_low_command.motor_cmd()[mapped].kp() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].kd() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
            continue;
        }

        this->unitree_low_command.motor_cmd()[mapped].mode() = 0x01;
        this->unitree_low_command.motor_cmd()[mapped].q() = command_local.motor_command.q[static_cast<size_t>(i)];
        this->unitree_low_command.motor_cmd()[mapped].dq() = command_local.motor_command.dq[static_cast<size_t>(i)];
        this->unitree_low_command.motor_cmd()[mapped].kp() = command_local.motor_command.kp[static_cast<size_t>(i)];
        this->unitree_low_command.motor_cmd()[mapped].kd() = command_local.motor_command.kd[static_cast<size_t>(i)];
        this->unitree_low_command.motor_cmd()[mapped].tau() = command_local.motor_command.tau[static_cast<size_t>(i)];
    }
    if (this->lowcmd_publisher)
    {
        this->unitree_low_command.crc() = this->Crc32Core(
            (uint32_t *)&this->unitree_low_command,
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        this->lowcmd_publisher->Write(this->unitree_low_command);
    }
}

void RL_Real_Go2X5::ExecuteSafeShutdownSequence()
{
    const int num_dofs = GetNumDofs();
    const auto default_pos = GetDefaultDofPos();
    std::vector<float> start_pose = default_pos;
    if (start_pose.size() != static_cast<size_t>(num_dofs))
    {
        start_pose.assign(static_cast<size_t>(num_dofs), 0.0f);
    }
    if (this->robot_state.motor_state.q.size() >= static_cast<size_t>(num_dofs))
    {
        for (int i = 0; i < num_dofs; ++i)
        {
            const float q = this->robot_state.motor_state.q[static_cast<size_t>(i)];
            if (std::isfinite(q))
            {
                start_pose[static_cast<size_t>(i)] = q;
            }
        }
    }
    auto target_pose = BuildSafeShutdownTargetPose(default_pos);

    if (target_pose.size() != static_cast<size_t>(num_dofs))
    {
        this->safe_shutdown_done = true;
        return;
    }

    auto kp = GetFixedKp();
    auto kd = GetFixedKd();
    if (kp.size() < static_cast<size_t>(num_dofs))
    {
        kp.resize(static_cast<size_t>(num_dofs), 40.0f);
    }
    if (kd.size() < static_cast<size_t>(num_dofs))
    {
        kd.resize(static_cast<size_t>(num_dofs), 3.0f);
    }

    const float soft_land_sec = std::max(0.2f, config_->GetShutdownSoftLandSec());
    const float hold_sec = std::max(0.0f, config_->GetShutdownHoldSec());
    const int step_ms = 10;
    const int land_steps = std::max(1, static_cast<int>(std::lround((soft_land_sec * 1000.0f) / static_cast<float>(step_ms))));
    const int hold_steps = std::max(1, static_cast<int>(std::lround((hold_sec * 1000.0f) / static_cast<float>(step_ms))));

    std::cout << "[Shutdown] Soft land + arm retract (land=" << soft_land_sec
              << "s, hold=" << hold_sec << "s)" << std::endl;

    for (int step = 1; step <= land_steps; ++step)
    {
        const float alpha = static_cast<float>(step) / static_cast<float>(land_steps);
        std::vector<float> pose(static_cast<size_t>(num_dofs), 0.0f);
        for (int i = 0; i < num_dofs; ++i)
        {
            const float q0 = start_pose[static_cast<size_t>(i)];
            const float q1 = target_pose[static_cast<size_t>(i)];
            pose[static_cast<size_t>(i)] = (1.0f - alpha) * q0 + alpha * q1;
        }
        this->PublishWholeBodyPose(pose, kp, kd);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }

    for (int i = 0; i < hold_steps; ++i)
    {
        this->PublishWholeBodyPose(target_pose, kp, kd);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        const int arm_size = std::max(0, std::min(this->arm_joint_count, num_dofs));
        if (arm_size > 0 &&
            this->arm_joint_start_index >= 0 &&
            (this->arm_joint_start_index + arm_size) <= num_dofs)
        {
            this->arm_hold_position.assign(
                target_pose.begin() + static_cast<long>(this->arm_joint_start_index),
                target_pose.begin() + static_cast<long>(this->arm_joint_start_index + arm_size));
            this->arm_joint_command_latest = this->arm_hold_position;
            this->arm_topic_command_latest = this->arm_hold_position;
            this->arm_topic_command_received = true;
            this->arm_hold_enabled = true;
            const uint64_t now_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            this->arm_joint_command_source_monotonic_ns_ = now_ns;
            this->arm_joint_command_publish_monotonic_ns_ = now_ns;
            this->arm_joint_command_expire_ns_ = 0;
            ++this->arm_joint_command_seq_;
        }
    }

    this->safe_shutdown_done = true;
}

void RL_Real_Go2X5::SafeShutdownNow()
{
    std::lock_guard<std::mutex> guard(this->safe_shutdown_mutex);
    if (this->safe_shutdown_done)
    {
        return;
    }

    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_control) this->loop_control->shutdown();
    if (this->loop_rl) this->loop_rl->shutdown();
#ifdef PLOT
    if (this->loop_plot) this->loop_plot->shutdown();
#endif

    try
    {
        this->arm_safe_shutdown_active.store(true);
        if (this->ShouldExecuteActiveShutdown())
        {
            this->ExecuteSafeShutdownSequence();
        }
        else
        {
            std::cout << "[Shutdown] Passive shutdown: body command already inactive, skip soft land."
                      << std::endl;
            const uint64_t now_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            if (this->unitree_adapter_active_ && this->unitree_adapter_)
            {
                const auto body_frame =
                    this->BuildHoldBodyCommandFrame(now_ns, Go2X5Supervisor::Mode::Passive);
                this->unitree_adapter_->SetCommand(body_frame);
                this->unitree_adapter_->ProcessCommand();
            }
            else
            {
                const int num_dofs = GetNumDofs();
                const auto joint_mapping = GetJointMapping();
                for (int i = 0; i < num_dofs && i < static_cast<int>(joint_mapping.size()); ++i)
                {
                    const int mapped = joint_mapping[static_cast<size_t>(i)];
                    if (mapped < 0 || mapped >= 20)
                    {
                        continue;
                    }
                    this->unitree_low_command.motor_cmd()[mapped].mode() = 0x00;
                    this->unitree_low_command.motor_cmd()[mapped].q() = PosStopF;
                    this->unitree_low_command.motor_cmd()[mapped].dq() = VelStopF;
                    this->unitree_low_command.motor_cmd()[mapped].kp() = 0.0f;
                    this->unitree_low_command.motor_cmd()[mapped].kd() = 0.0f;
                    this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
                }
                if (this->lowcmd_publisher)
                {
                    this->unitree_low_command.crc() = this->Crc32Core(
                        (uint32_t *)&this->unitree_low_command,
                        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
                    this->lowcmd_publisher->Write(this->unitree_low_command);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "[WARNING] Safe shutdown sequence failed: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "[WARNING] Safe shutdown sequence failed with unknown error." << std::endl;
    }
    this->arm_safe_shutdown_active.store(false);
    this->CloseArmCommandIpc();
    this->CloseArmBridgeIpc();
    this->safe_shutdown_done = true;
}
