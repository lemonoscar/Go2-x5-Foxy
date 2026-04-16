
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/control_logic.hpp"
#include "rl_sar/go2x5/ipc.hpp"
#include <cmath>
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

Go2X5Supervisor::Mode RL_Real_Go2X5::GetSupervisorModeSnapshot() const
{
    std::lock_guard<std::mutex> lock(this->supervisor_mutex);
    if (this->supervisor_)
    {
        return this->supervisor_->mode();
    }
    return Go2X5Supervisor::Mode::Boot;
}

bool RL_Real_Go2X5::ShouldActuateArmForMode(const Go2X5Supervisor::Mode mode) const
{
    switch (mode)
    {
    case Go2X5Supervisor::Mode::RlDogOnlyActive:
    case Go2X5Supervisor::Mode::ManualArm:
        return true;
    case Go2X5Supervisor::Mode::Boot:
    case Go2X5Supervisor::Mode::Probe:
    case Go2X5Supervisor::Mode::Passive:
    case Go2X5Supervisor::Mode::Ready:
    case Go2X5Supervisor::Mode::DegradedArm:
    case Go2X5Supervisor::Mode::DegradedBody:
    case Go2X5Supervisor::Mode::SoftStop:
    case Go2X5Supervisor::Mode::FaultLatched:
        return false;
    }
    return false;
}

bool RL_Real_Go2X5::ShouldApplyActiveArmControl(const Go2X5Supervisor::Mode mode) const
{
    if (!this->ShouldActuateArmForMode(mode))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    return this->arm_explicit_command_active_;
}

bool RL_Real_Go2X5::ShouldExecuteActiveShutdown() const
{
    const auto mode = this->GetSupervisorModeSnapshot();
    switch (mode)
    {
    case Go2X5Supervisor::Mode::RlDogOnlyActive:
    case Go2X5Supervisor::Mode::ManualArm:
    case Go2X5Supervisor::Mode::DegradedArm:
    case Go2X5Supervisor::Mode::DegradedBody:
        return true;
    case Go2X5Supervisor::Mode::Boot:
    case Go2X5Supervisor::Mode::Probe:
    case Go2X5Supervisor::Mode::Passive:
    case Go2X5Supervisor::Mode::Ready:
    case Go2X5Supervisor::Mode::SoftStop:
    case Go2X5Supervisor::Mode::FaultLatched:
        break;
    }

    const int num_dofs = this->GetNumDofs();
    if (num_dofs <= 0 ||
        this->robot_command.motor_command.q.size() < static_cast<size_t>(num_dofs) ||
        this->robot_command.motor_command.dq.size() < static_cast<size_t>(num_dofs) ||
        this->robot_command.motor_command.kp.size() < static_cast<size_t>(num_dofs) ||
        this->robot_command.motor_command.kd.size() < static_cast<size_t>(num_dofs) ||
        this->robot_command.motor_command.tau.size() < static_cast<size_t>(num_dofs))
    {
        return false;
    }

    for (int i = 0; i < num_dofs; ++i)
    {
        if (this->IsArmJointIndex(i))
        {
            continue;
        }

        const size_t idx = static_cast<size_t>(i);
        const bool passive_joint =
            this->robot_command.motor_command.q[idx] == static_cast<float>(PosStopF) &&
            this->robot_command.motor_command.dq[idx] == static_cast<float>(VelStopF) &&
            std::fabs(this->robot_command.motor_command.kp[idx]) <= 1e-6f &&
            std::fabs(this->robot_command.motor_command.kd[idx]) <= 1e-6f &&
            std::fabs(this->robot_command.motor_command.tau[idx]) <= 1e-6f;
        if (!passive_joint)
        {
            return true;
        }
    }

    return false;
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

    double control_frequency_hz = 0.0;
    {
        std::lock_guard<std::mutex> lock(this->runtime_metrics_mutex);
        control_frequency_hz = this->last_coordinator_frequency_hz_;
    }

    if (this->policy_inference_log_enabled &&
        (this->last_policy_inference_hz > 0.0f || control_frequency_hz > 0.0))
    {
        std::cout << "\r\033[K" << LOGGER::INFO
                  << "Control frequency: " << std::fixed << std::setprecision(2) << control_frequency_hz
                  << " Hz | Policy inference frequency: " << this->last_policy_inference_hz
                  << " Hz" << std::flush;
    }

    this->RefreshSupervisorState("policy_inference");
}
