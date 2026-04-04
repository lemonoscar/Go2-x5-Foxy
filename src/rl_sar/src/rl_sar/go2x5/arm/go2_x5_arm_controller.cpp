
#include "rl_sar/go2x5/arm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace Go2X5ArmController {

// ============================================================================
// ArmController Implementation
// ============================================================================

ArmController::ArmController() = default;

ArmController::~ArmController() {
    Shutdown();
}

ControllerResult ArmController::Initialize(
    const ControllerConfig& config,
    Go2X5State::StateManager& state_manager)
{
    config_ = config;
    state_manager_ = &state_manager;

    // Initialize StateManager config
    state_manager_->SetConfig(
        config_.arm_joint_count + (config_.arm_joint_start_index > 0 ? config_.arm_joint_start_index : 12),
        config_.arm_joint_start_index,
        config_.arm_joint_count,
        config_.arm_control_mode
    );

    // Initialize arm command state
    const int arm_size = config_.arm_command_size > 0 ? config_.arm_command_size : config_.arm_joint_count;

    // Set initial hold position
    std::vector<float> initial_hold;
    if (!config_.arm_hold_pose.empty() &&
        static_cast<int>(config_.arm_hold_pose.size()) == arm_size) {
        initial_hold = config_.arm_hold_pose;
    } else if (!config_.default_dof_pos.empty()) {
        const int start = std::max(0, config_.arm_joint_start_index);
        const int end = std::min(
            static_cast<int>(config_.default_dof_pos.size()),
            start + arm_size
        );
        if (end > start) {
            initial_hold.assign(
                config_.default_dof_pos.begin() + start,
                config_.default_dof_pos.begin() + end
            );
        }
    }

    if (initial_hold.size() != static_cast<size_t>(arm_size)) {
        initial_hold.assign(static_cast<size_t>(arm_size), 0.0f);
    }

    state_manager_->SetArmHoldPosition(initial_hold);
    state_manager_->SetArmCommandSmoothed(initial_hold);
    state_manager_->SetArmJointCommandLatest(initial_hold);
    state_manager_->SetArmTopicCommandLatest(initial_hold);
    state_manager_->SetArmHoldEnabled(config_.arm_hold_enabled);

    // Calculate smoothing ticks
    const float step_dt = config_.dt * config_.decimation;
    const int smoothing_ticks = static_cast<int>(
        std::round(config_.arm_command_smoothing_time / step_dt)
    );
    state_manager_->SetArmCommandSmoothingTicks(smoothing_ticks);
    state_manager_->SetArmCommandSmoothingCounter(0);

    // Initialize safety context
    UpdateSafetyContext();

    initialized_ = true;
    return ControllerResult::OK;
}

bool ArmController::InitializeTransport(int cmd_port, int state_port, const std::string& host) {
    if (transport_) {
        transport_->Shutdown();
    }

    transport_ = Go2X5ArmTransport::TransportFactory::CreateIpcTransport(
        cmd_port, state_port, host
    );

    std::string error;
    if (!transport_->Initialize(&error)) {
        return false;
    }

    state_manager_->SetArmBridgeConnected(true);
    return true;
}

void ArmController::Shutdown() {
    if (transport_) {
        transport_->Shutdown();
        transport_.reset();
    }
    state_manager_->SetArmBridgeConnected(false);
    initialized_ = false;
}

ControllerResult ArmController::SetHoldTarget(const std::vector<float>& target, const char* reason) {
    if (!initialized_) {
        return ControllerResult::NOT_INITIALIZED;
    }

    const int arm_size = config_.arm_command_size > 0 ? config_.arm_command_size : config_.arm_joint_count;

    if (static_cast<int>(target.size()) != arm_size) {
        return ControllerResult::INVALID_SIZE;
    }

    std::vector<float> target_copy = target;
    std::vector<float> fallback = state_manager_->GetArmHoldPosition();

    if (!ClipArmPoseTarget(target_copy, fallback, reason)) {
        return ControllerResult::LIMIT_EXCEEDED;
    }

    // Update smoothing
    auto start = state_manager_->GetArmCommandSmoothingStart();
    state_manager_->SetArmCommandSmoothingStart(state_manager_->GetArmCommandSmoothed());
    state_manager_->SetArmCommandSmoothingTarget(target_copy);
    state_manager_->SetArmCommandSmoothingCounter(state_manager_->GetArmCommandSmoothingTicks());
    state_manager_->SetArmHoldPosition(target_copy);

    return ControllerResult::OK;
}

ControllerResult ArmController::UpdateFromTopicCommand(const std::vector<float>& data, const char* context) {
    if (!initialized_) {
        return ControllerResult::NOT_INITIALIZED;
    }

    const int arm_size = config_.arm_command_size > 0 ? config_.arm_command_size : config_.arm_joint_count;

    if (static_cast<int>(data.size()) < arm_size) {
        return ControllerResult::INVALID_SIZE;
    }

    std::vector<float> target(data.begin(), data.begin() + arm_size);
    std::vector<float> fallback = state_manager_->GetArmHoldPosition();

    if (!ClipArmPoseTarget(target, fallback, context)) {
        return ControllerResult::LIMIT_EXCEEDED;
    }

    state_manager_->SetArmJointCommandLatest(target);
    state_manager_->SetArmTopicCommandLatest(target);
    state_manager_->SetArmTopicCommandReceived(true);

    return ControllerResult::OK;
}

std::vector<float> ArmController::StepSmoothing(const std::vector<float>& desired_command) {
    if (!initialized_) {
        return {};
    }

    const int counter = state_manager_->GetArmCommandSmoothingCounter();
    const int ticks = state_manager_->GetArmCommandSmoothingTicks();

    if (counter <= 0 || ticks <= 0) {
        // No smoothing active, return desired directly
        state_manager_->SetArmCommandSmoothed(desired_command);
        return desired_command;
    }

    const std::vector<float> start = state_manager_->GetArmCommandSmoothingStart();
    const std::vector<float> target = state_manager_->GetArmCommandSmoothingTarget();

    if (start.empty() || target.size() != desired_command.size()) {
        // Invalid smoothing state, return desired directly
        state_manager_->SetArmCommandSmoothingCounter(0);
        state_manager_->SetArmCommandSmoothed(desired_command);
        return desired_command;
    }

    // Interpolate
    const float alpha = 1.0f - static_cast<float>(counter) / static_cast<float>(ticks);
    std::vector<float> smoothed(desired_command.size(), 0.0f);

    for (size_t i = 0; i < smoothed.size(); ++i) {
        smoothed[i] = (1.0f - alpha) * start[i] + alpha * target[i];
    }

    state_manager_->SetArmCommandSmoothingCounter(counter - 1);
    state_manager_->SetArmCommandSmoothed(smoothed);

    return smoothed;
}

std::vector<float> ArmController::GetSmoothedCommand() const {
    return state_manager_->GetArmCommandSmoothed();
}

void ArmController::SetHoldEnabled(bool enabled) {
    state_manager_->SetArmHoldEnabled(enabled);
}

bool ArmController::IsHoldEnabled() const {
    return state_manager_->IsArmHoldEnabled();
}

ControllerResult ArmController::SendBridgeCommand(const ArmBridgeCommand& cmd) {
    if (!initialized_ || !transport_) {
        return ControllerResult::NOT_INITIALIZED;
    }

    if (!cmd.is_valid()) {
        return ControllerResult::INVALID_SIZE;
    }

    Go2X5ArmTransport::ArmCommand transport_cmd;
    transport_cmd.q = cmd.q;
    transport_cmd.dq = cmd.dq;
    transport_cmd.kp = cmd.kp;
    transport_cmd.kd = cmd.kd;
    transport_cmd.tau = cmd.tau;

    std::string error;
    if (!transport_->SendCommand(transport_cmd, &error)) {
        return ControllerResult::TRANSPORT_ERROR;
    }

    return ControllerResult::OK;
}

bool ArmController::ReceiveBridgeState() {
    if (!initialized_ || !transport_) {
        return false;
    }

    Go2X5ArmTransport::ArmState state;
    std::string error;
    if (!transport_->ReceiveState(&state, &error)) {
        return false;
    }

    // Update StateManager
    state_manager_->SetArmBridgeState(
        state.q, state.dq, state.tau, state.from_backend
    );

    return true;
}

bool ArmController::IsBridgeStateFresh(double max_age_ms) const {
    if (!initialized_) {
        return false;
    }
    return state_manager_->IsArmBridgeStateFresh(max_age_ms);
}

bool ArmController::IsBridgeStateValid() const {
    if (!initialized_) {
        return false;
    }
    return state_manager_->IsArmBridgeStateValid();
}

bool ArmController::GetBridgeState(std::vector<float>* q, std::vector<float>* dq, std::vector<float>* tau) const {
    if (!initialized_) {
        return false;
    }
    return state_manager_->GetArmBridgeState(q, dq, tau);
}

ControllerStateSnapshot ArmController::CaptureSnapshot() const {
    ControllerStateSnapshot snapshot;

    snapshot.arm_hold_position = state_manager_->GetArmHoldPosition();
    snapshot.arm_command_smoothed = state_manager_->GetArmCommandSmoothed();
    snapshot.arm_joint_command_latest = state_manager_->GetArmJointCommandLatest();
    snapshot.arm_topic_command_latest = state_manager_->GetArmTopicCommandLatest();
    snapshot.arm_topic_command_received = state_manager_->HasArmTopicCommand();
    snapshot.arm_command_smoothing_counter = state_manager_->GetArmCommandSmoothingCounter();
    snapshot.arm_command_smoothing_ticks = state_manager_->GetArmCommandSmoothingTicks();

    state_manager_->GetArmBridgeState(
        &snapshot.arm_bridge_state_q,
        &snapshot.arm_bridge_state_dq,
        &snapshot.arm_bridge_state_tau
    );
    snapshot.arm_bridge_state_valid = state_manager_->IsArmBridgeStateValid();
    snapshot.arm_bridge_state_stamp = state_manager_->GetArmBridgeStateStamp();

    return snapshot;
}

void ArmController::RestoreFromSnapshot(const ControllerStateSnapshot& snapshot) {
    state_manager_->SetArmHoldPosition(snapshot.arm_hold_position);
    state_manager_->SetArmCommandSmoothed(snapshot.arm_command_smoothed);
    state_manager_->SetArmJointCommandLatest(snapshot.arm_joint_command_latest);
    state_manager_->SetArmTopicCommandLatest(snapshot.arm_topic_command_latest);
    state_manager_->SetArmTopicCommandReceived(snapshot.arm_topic_command_received);
    state_manager_->SetArmCommandSmoothingCounter(snapshot.arm_command_smoothing_counter);
    state_manager_->SetArmCommandSmoothingTicks(snapshot.arm_command_smoothing_ticks);

    if (snapshot.arm_bridge_state_valid) {
        state_manager_->SetArmBridgeState(
            snapshot.arm_bridge_state_q,
            snapshot.arm_bridge_state_dq,
            snapshot.arm_bridge_state_tau,
            snapshot.arm_bridge_state_from_backend
        );
    }
}

std::vector<float> ArmController::GetHoldPosition() const {
    return state_manager_->GetArmHoldPosition();
}

bool ArmController::ClipArmPoseTarget(std::vector<float>& target,
                                      const std::vector<float>& fallback,
                                      const char* context) const {
    return Go2X5SafetyGuard::ClipArmPoseTargetInPlace(
        &target, fallback, safety_context_, context
    );
}

bool ArmController::ClipBridgeCommand(ArmBridgeCommand& cmd,
                                      const std::vector<float>& fallback,
                                      const char* context) const {
    return Go2X5SafetyGuard::ClipArmBridgeCommandInPlace(
        &cmd.q, &cmd.dq, &cmd.kp, &cmd.kd, &cmd.tau,
        fallback, safety_context_, context
    );
}

void ArmController::PollCommandIpc() {
#if defined(__linux__)
    if (command_ipc_fd_ < 0) {
        return;
    }

    const int arm_size = config_.arm_command_size > 0 ? config_.arm_command_size : config_.arm_joint_count;
    if (arm_size <= 0) {
        return;
    }

    std::array<uint8_t, 1024> buffer{};
    while (true) {
        const ssize_t bytes_read = recv(command_ipc_fd_, buffer.data(), buffer.size(), 0);
        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            break;
        }
        if (bytes_read == 0) {
            break;
        }

        std::vector<uint8_t> bytes(buffer.begin(), buffer.begin() + bytes_read);
        Go2X5IPC::ArmPosePacket packet;
        std::string error;
        if (!Go2X5IPC::ParsePosePacket(bytes, packet, &error)) {
            continue;
        }
        if (packet.joint_count != static_cast<uint16_t>(arm_size)) {
            continue;
        }

        UpdateFromTopicCommand(packet.q, "Arm joint command IPC");
    }
#endif
}

void ArmController::PollBridgeStateIpc() {
#if defined(__linux__)
    if (bridge_state_fd_ < 0) {
        return;
    }

    const int arm_size = config_.arm_joint_count;
    if (arm_size <= 0) {
        return;
    }

    std::array<uint8_t, 2048> buffer{};
    while (true) {
        const ssize_t bytes_read = recv(bridge_state_fd_, buffer.data(), buffer.size(), 0);
        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            break;
        }
        if (bytes_read == 0) {
            break;
        }

        std::vector<uint8_t> bytes(buffer.begin(), buffer.begin() + bytes_read);
        Go2X5IPC::ArmStatePacket packet;
        std::string error;
        if (!Go2X5IPC::ParseStatePacket(bytes, packet, &error)) {
            continue;
        }
        if (packet.joint_count != static_cast<uint16_t>(arm_size)) {
            continue;
        }

        state_manager_->SetArmBridgeState(
            packet.q, packet.dq, packet.tau, packet.state_from_backend
        );
    }
#endif
}

void ArmController::SetCommandIpcSocket(int fd) {
    command_ipc_fd_ = fd;
}

void ArmController::SetBridgeIpcSockets(int cmd_fd, int state_fd) {
    bridge_cmd_fd_ = cmd_fd;
    bridge_state_fd_ = state_fd;
}

void ArmController::UpdateShadowState(const std::vector<float>& q, const std::vector<float>& dq) {
    // Update shadow state for fallback
    // This is used when bridge state is unavailable
    state_manager_->SetArmBridgeState(q, dq, std::vector<float>(q.size(), 0.0f), false);
}

bool ArmController::ReadArmState(std::vector<float>* state_q,
                                 std::vector<float>* state_dq,
                                 std::vector<float>* state_tau) {
    if (!initialized_) {
        return false;
    }

    // Poll for new state if using IPC
    if (bridge_state_fd_ >= 0) {
        PollBridgeStateIpc();
    } else if (transport_) {
        ReceiveBridgeState();
    }

    // Check if we need to require live state
    const bool require_live = config_.arm_bridge_require_live_state;
    const bool is_fresh = IsBridgeStateFresh(config_.arm_bridge_state_timeout_sec * 1000.0);
    const bool is_valid = IsBridgeStateValid();

    if (require_live && (!is_fresh || !is_valid)) {
        // State is required but not available
        return false;
    }

    return GetBridgeState(state_q, state_dq, state_tau);
}

Go2X5ArmTransport::TransportStats ArmController::GetTransportStats() const {
    if (transport_) {
        return transport_->GetStats();
    }
    return Go2X5ArmTransport::TransportStats{};
}

void ArmController::ResetTransportStats() {
    if (transport_) {
        transport_->ResetStats();
    }
}

void ArmController::UpdateSafetyContext() {
    safety_context_.arm_joint_start_index = config_.arm_joint_start_index;
    safety_context_.arm_joint_count = config_.arm_joint_count;
    safety_context_.arm_joint_lower_limits = config_.arm_joint_lower_limits;
    safety_context_.arm_joint_upper_limits = config_.arm_joint_upper_limits;
    safety_context_.whole_body_velocity_limits = config_.whole_body_velocity_limits;
    safety_context_.whole_body_effort_limits = config_.whole_body_effort_limits;
    safety_context_.whole_body_kp_limits = config_.whole_body_kp_limits;
    safety_context_.whole_body_kd_limits = config_.whole_body_kd_limits;
}

} // namespace Go2X5ArmController
