#include "rl_sar/adapters/unitree_adapter.hpp"

// Unitree SDK2 headers - these must be included after our header
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace unitree::robot;

// Topic constants for Go2 DDS communication
constexpr const char* TOPIC_LOWCMD = "rt/lowcmd";
constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";

// Unitree low-level command constants
constexpr double kPosStopF = (2.146E+9f);
constexpr double kVelStopF = (16000.0f);
constexpr uint8_t kLowCmdHeader0 = 0xFE;
constexpr uint8_t kLowCmdHeader1 = 0xEF;
constexpr uint8_t kLowCmdLevelFlag = 0xFF;
constexpr uint8_t kMotorModePseudoContinuous = 0x01;
constexpr uint8_t kMotorModeDisabled = 0x00;

// Unitree Go2 has 20 motors total (12 leg + 6 arm + 2 waist)
constexpr int kUnitreeMotorCount = 20;
constexpr float kQuaternionNormEpsilon = 1e-6f;

std::array<float, 4> NormalizeQuaternion(const std::array<float, 4>& quat)
{
    float norm_sq = 0.0f;
    for (float value : quat)
    {
        if (!std::isfinite(value))
        {
            return {1.0f, 0.0f, 0.0f, 0.0f};
        }
        norm_sq += value * value;
    }

    if (norm_sq <= kQuaternionNormEpsilon)
    {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    const float inv_norm = 1.0f / std::sqrt(norm_sq);
    return {
        quat[0] * inv_norm,
        quat[1] * inv_norm,
        quat[2] * inv_norm,
        quat[3] * inv_norm,
    };
}

std::array<float, 3> RotateInverse(const std::array<float, 4>& quat,
                                   const std::array<float, 3>& vec)
{
    const auto q = NormalizeQuaternion(quat);
    const float q_w = q[0];
    const float q_x = q[1];
    const float q_y = q[2];
    const float q_z = q[3];

    const float v_x = vec[0];
    const float v_y = vec[1];
    const float v_z = vec[2];

    const float a_x = v_x * (2.0f * q_w * q_w - 1.0f);
    const float a_y = v_y * (2.0f * q_w * q_w - 1.0f);
    const float a_z = v_z * (2.0f * q_w * q_w - 1.0f);

    const float cross_x = q_y * v_z - q_z * v_y;
    const float cross_y = q_z * v_x - q_x * v_z;
    const float cross_z = q_x * v_y - q_y * v_x;

    const float b_x = cross_x * q_w * 2.0f;
    const float b_y = cross_y * q_w * 2.0f;
    const float b_z = cross_z * q_w * 2.0f;

    const float dot = q_x * v_x + q_y * v_y + q_z * v_z;

    const float c_x = q_x * dot * 2.0f;
    const float c_y = q_y * dot * 2.0f;
    const float c_z = q_z * dot * 2.0f;

    return {a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z};
}

bool IsPassiveBodyJointCommand(const float q,
                               const float dq,
                               const float kp,
                               const float kd,
                               const float tau)
{
    return q == static_cast<float>(kPosStopF) &&
           dq == static_cast<float>(kVelStopF) &&
           std::fabs(kp) <= 1e-6f &&
           std::fabs(kd) <= 1e-6f &&
           std::fabs(tau) <= 1e-6f;
}

uint32_t Crc32Core(const uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t crc32 = 0xFFFFFFFF;
    constexpr uint32_t kPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; ++i)
    {
        xbit = 1u << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; ++bits)
        {
            if (crc32 & 0x80000000u)
            {
                crc32 <<= 1;
                crc32 ^= kPolynomial;
            }
            else
            {
                crc32 <<= 1;
            }
            if (data & xbit)
            {
                crc32 ^= kPolynomial;
            }
            xbit >>= 1;
        }
    }
    return crc32;
}

namespace rl_sar::adapters
{

// ============================================================================
// PIMPL Implementation for DDS types (header isolation)
// ============================================================================

struct UnitreeAdapter::LowCmdPublisher
{
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> publisher;
};

struct UnitreeAdapter::LowStateSubscriber
{
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> subscriber;
};

// ============================================================================
// Construction/Destruction
// ============================================================================

UnitreeAdapter::UnitreeAdapter()
    : state_stamp_{}
    , last_write_stamp_{}
    , source_monotonic_ns_(0)
{
    // Initialize pending command with safe defaults
    pending_command_.header.msg_type = protocol::FrameType::BodyCommand;
    pending_command_.header.version = protocol::kProtocolVersion;
    pending_command_.header.magic = protocol::kFrameMagic;
    pending_command_.joint_count = protocol::kDogJointCount;
    pending_command_.q.fill(0.0f);
    pending_command_.dq.fill(0.0f);
    pending_command_.kp.fill(0.0f);
    pending_command_.kd.fill(0.0f);
    pending_command_.tau.fill(0.0f);

    // Initialize latest state with safe defaults
    latest_state_.header.msg_type = protocol::FrameType::BodyState;
    latest_state_.header.version = protocol::kProtocolVersion;
    latest_state_.header.magic = protocol::kFrameMagic;
    latest_state_.imu_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    latest_state_.imu_gyro.fill(0.0f);
    latest_state_.imu_acc.fill(0.0f);
    latest_state_.leg_q.fill(0.0f);
    latest_state_.leg_dq.fill(0.0f);
    latest_state_.leg_tau.fill(0.0f);
    latest_state_.base_lin_vel.fill(0.0f);
    latest_state_.base_ang_vel.fill(0.0f);
    latest_state_.projected_gravity.fill(0.0f);
    latest_state_.lowstate_age_us = 0;
    latest_state_.dds_ok = 0;

    // Initialize velocity estimation buffers
    foot_force_.fill(0.0f);
    leg_position_buffer_.fill(0.0f);
    leg_velocity_buffer_.fill(0.0f);
}

UnitreeAdapter::~UnitreeAdapter()
{
    Stop();
}

// ============================================================================
// Initialization and Lifecycle
// ============================================================================

UnitreeAdapter::Status UnitreeAdapter::Initialize(const Config& config)
{
    if (initialized_.load(std::memory_order_acquire))
    {
        return Status::kAlreadyInitialized;
    }

    // Validate configuration
    if (config.network_interface.empty())
    {
        return Status::kInvalidConfig;
    }
    if (config.command_rate_hz <= 0 || config.command_rate_hz > 1000)
    {
        return Status::kInvalidConfig;
    }
    if (config.leg_dof_count != protocol::kDogJointCount)
    {
        return Status::kInvalidConfig;
    }
    std::array<bool, kUnitreeMotorCount> seen_mapping{};
    for (int motor_index : config.joint_mapping)
    {
        if (motor_index < 0 || motor_index >= kUnitreeMotorCount)
        {
            return Status::kInvalidConfig;
        }
        if (seen_mapping[static_cast<size_t>(motor_index)])
        {
            return Status::kInvalidConfig;
        }
        seen_mapping[static_cast<size_t>(motor_index)] = true;
    }

    config_ = config;

    // Create velocity estimator if enabled
    if (config_.enable_velocity_estimation)
    {
        velocity_estimator_ = std::make_unique<state_estimation::VelocityEstimator>(
            config_.velocity_estimator_config);
    }

    try
    {
        // Initialize Unitree SDK2 ChannelFactory
        // Note: ChannelFactory::Instance()->Init() should be called once per process
        // We assume it's initialized by the main application
        // If not initialized, we attempt to initialize it here
        if (config_.initialize_channel_factory)
        {
            ChannelFactory::Instance()->Init(0, config_.network_interface.c_str());
        }

        // Create lowcmd publisher
        lowcmd_publisher_ = std::make_unique<LowCmdPublisher>();
        lowcmd_publisher_->publisher.reset(
            new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher_->publisher->InitChannel();

        // Create lowstate subscriber
        lowstate_subscriber_ = std::make_unique<LowStateSubscriber>();
        lowstate_subscriber_->subscriber.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

        // Bind the callback with queue size of 1 (always get latest)
        lowstate_subscriber_->subscriber->InitChannel(
            [this](const void* msg) { this->LowStateMessageHandler(msg); },
            1);

        // Allocate lowcmd buffer
        lowcmd_buffer_ = std::make_unique<unitree_go::msg::dds_::LowCmd_>();
        InitializeLowCmd();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[UnitreeAdapter] Initialization failed: " << e.what() << std::endl;
        lowcmd_publisher_.reset();
        lowstate_subscriber_.reset();
        lowcmd_buffer_.reset();
        return Status::kDdsInitFailed;
    }

    // Initialize source timestamp
    source_monotonic_ns_ = GetMonotonicNs();

    initialized_.store(true, std::memory_order_release);
    return Status::kOk;
}

UnitreeAdapter::Status UnitreeAdapter::Start()
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return Status::kNotInitialized;
    }

    if (started_.load(std::memory_order_acquire))
    {
        return Status::kAlreadyStarted;
    }

    // Reset diagnostic counters
    write_fail_count_.store(0, std::memory_order_relaxed);
    seq_gap_count_.store(0, std::memory_order_relaxed);
    lowstate_seen_.store(false, std::memory_order_release);
    lowstate_seq_.store(0, std::memory_order_release);
    command_seq_.store(0, std::memory_order_release);
    velocity_estimator_ready_ = false;
    if (velocity_estimator_)
    {
        velocity_estimator_->Reset();
    }

    started_.store(true, std::memory_order_release);
    return Status::kOk;
}

void UnitreeAdapter::Stop()
{
    if (!started_.load(std::memory_order_acquire))
    {
        return;
    }

    started_.store(false, std::memory_order_release);

    // Send zero commands before shutdown to ensure safe stop
    if (lowcmd_publisher_ && lowcmd_buffer_)
    {
        InitializeLowCmd();
        lowcmd_buffer_->crc() = ComputeLowCmdCrc(*lowcmd_buffer_);
        try
        {
            if (lowcmd_publisher_->publisher)
            {
                lowcmd_publisher_->publisher->Write(*lowcmd_buffer_);
            }
        }
        catch (...)
        {
            // Ignore errors during shutdown
        }
    }

    // Clean up DDS resources
    lowcmd_publisher_.reset();
    lowstate_subscriber_.reset();
    lowcmd_buffer_.reset();
}

// ============================================================================
// Command Processing
// ============================================================================

UnitreeAdapter::Status UnitreeAdapter::SetCommand(const protocol::BodyCommandFrame& cmd)
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return Status::kNotInitialized;
    }

    if (!started_.load(std::memory_order_acquire))
    {
        return Status::kNotStarted;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_command_ = cmd;
    return Status::kOk;
}

UnitreeAdapter::Status UnitreeAdapter::ProcessCommand()
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return Status::kNotInitialized;
    }

    if (!started_.load(std::memory_order_acquire))
    {
        return Status::kNotStarted;
    }

    // Get pending command
    protocol::BodyCommandFrame cmd;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd = pending_command_;
    }

    // Update command frame metadata
    cmd.header.seq = command_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
    cmd.header.source_monotonic_ns = GetMonotonicNs();
    cmd.header.publish_monotonic_ns = cmd.header.source_monotonic_ns;
    cmd.header.source_id = config_.source_id;

    // Convert to LowCmd_ and write
    ConvertBodyCommandToLowCmd(cmd);

    try
    {
        if (lowcmd_publisher_ && lowcmd_publisher_->publisher)
        {
            lowcmd_publisher_->publisher->Write(*lowcmd_buffer_);
            last_write_stamp_ = std::chrono::steady_clock::now();
            return Status::kOk;
        }
        else
        {
            return Status::kWriteFailed;
        }
    }
    catch (const std::exception& e)
    {
        write_fail_count_.fetch_add(1, std::memory_order_relaxed);
        std::cerr << "[UnitreeAdapter] DDS write failed: " << e.what() << std::endl;
        return Status::kWriteFailed;
    }
}

// ============================================================================
// State Retrieval
// ============================================================================

bool UnitreeAdapter::GetState(protocol::BodyStateFrame& state)
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;

    // Update lowstate age based on current time
    const auto now = std::chrono::steady_clock::now();
    if (state_stamp_.time_since_epoch().count() > 0)
    {
        const auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - state_stamp_);
        state.lowstate_age_us = static_cast<uint32_t>(age.count());
        const uint64_t timeout_us = static_cast<uint64_t>(config_.lowstate_timeout_ms * 1000.0);
        if (static_cast<uint64_t>(state.lowstate_age_us) > timeout_us)
        {
            state.header.validity_flags |= protocol::kValidityStale;
        }
    }
    else
    {
        state.lowstate_age_us = UINT32_MAX;
        state.header.validity_flags |= protocol::kValidityStale;
    }

    return true;
}

bool UnitreeAdapter::IsStateStale() const
{
    const uint64_t age_us = GetStateAgeUs();
    const uint64_t timeout_us = static_cast<uint64_t>(config_.lowstate_timeout_ms * 1000.0);
    return age_us > timeout_us;
}

uint64_t UnitreeAdapter::GetStateAgeUs() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_stamp_.time_since_epoch().count() == 0)
    {
        return UINT64_MAX;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - state_stamp_);
    return static_cast<uint64_t>(age.count());
}

UnitreeAdapter::Diagnostics UnitreeAdapter::GetDiagnostics() const
{
    Diagnostics diag;
    diag.initialized = initialized_.load(std::memory_order_acquire);
    diag.started = started_.load(std::memory_order_acquire);
    diag.lowstate_seen = lowstate_seen_.load(std::memory_order_acquire);
    diag.dds_write_ok = write_fail_count_.load(std::memory_order_relaxed) == 0;
    diag.lowstate_seq = lowstate_seq_.load(std::memory_order_acquire);
    diag.command_seq = command_seq_.load(std::memory_order_acquire);
    diag.lowstate_age_us = const_cast<UnitreeAdapter*>(this)->GetStateAgeUs();
    diag.write_fail_count = write_fail_count_.load(std::memory_order_relaxed);
    diag.seq_gap_count = seq_gap_count_.load(std::memory_order_relaxed);

    // Calculate last write age
    if (last_write_stamp_.time_since_epoch().count() > 0)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - last_write_stamp_);
        diag.last_write_age_us = static_cast<uint64_t>(age.count());
    }
    else
    {
        diag.last_write_age_us = UINT64_MAX;
    }

    return diag;
}

// ============================================================================
// Internal DDS Callback
// ============================================================================

void UnitreeAdapter::LowStateMessageHandler(const void* message)
{
    const auto* msg = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    if (!msg)
    {
        return;
    }

    const uint64_t now_ns = GetMonotonicNs();

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Copy IMU data
    for (size_t i = 0; i < latest_state_.imu_quat.size(); ++i)
    {
        latest_state_.imu_quat[i] = msg->imu_state().quaternion()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < latest_state_.imu_gyro.size(); ++i)
    {
        latest_state_.imu_gyro[i] = msg->imu_state().gyroscope()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < latest_state_.imu_acc.size(); ++i)
    {
        latest_state_.imu_acc[i] = msg->imu_state().accelerometer()[static_cast<int>(i)];
    }
    latest_state_.base_ang_vel = latest_state_.imu_gyro;
    latest_state_.projected_gravity = RotateInverse(
        latest_state_.imu_quat, std::array<float, 3>{0.0f, 0.0f, -1.0f});

    // Copy motor state (we only care about the first 12 for leg DOFs)
    latest_state_.leg_q.fill(0.0f);
    latest_state_.leg_dq.fill(0.0f);
    latest_state_.leg_tau.fill(0.0f);
    foot_force_.fill(0.0f);

    for (int i = 0; i < protocol::kDogJointCount; ++i)
    {
        const int motor_index = ResolveMotorIndexForBodyJoint(i);
        if (motor_index < 0 || motor_index >= kUnitreeMotorCount)
        {
            continue;
        }
        const auto& motor_state = msg->motor_state()[motor_index];
        latest_state_.leg_q[static_cast<size_t>(i)] = motor_state.q();
        latest_state_.leg_dq[static_cast<size_t>(i)] = motor_state.dq();
        latest_state_.leg_tau[static_cast<size_t>(i)] = motor_state.tau_est();

        // Store in buffer for velocity estimator
        leg_position_buffer_[i] = motor_state.q();
        leg_velocity_buffer_[i] = motor_state.dq();
    }

    // Use the SDK-provided foot contact forces instead of inferred motor torques.
    // The project leg order is [FR, FL, RR, RL], matching both training and Unitree Go2.
    for (size_t leg_id = 0; leg_id < foot_force_.size(); ++leg_id)
    {
        const float estimated_force = std::fabs(static_cast<float>(msg->foot_force_est()[leg_id]));
        const float raw_force = std::fabs(static_cast<float>(msg->foot_force()[leg_id]));
        foot_force_[leg_id] =
            std::fabs(estimated_force) > 1e-6f ? estimated_force : raw_force;
    }

    // Estimate base linear velocity
    if (velocity_estimator_)
    {
        velocity_estimator_->Update(
            now_ns,
            latest_state_.imu_acc,
            latest_state_.imu_gyro,
            latest_state_.imu_quat,
            leg_position_buffer_,
            leg_velocity_buffer_,
            foot_force_
        );
        latest_state_.base_lin_vel = velocity_estimator_->GetEstimatedVelocity();
        velocity_estimator_ready_ = velocity_estimator_->IsReady();
    }
    else
    {
        // Velocity estimation disabled
        latest_state_.base_lin_vel = {0.0f, 0.0f, 0.0f};
    }

    // Update frame metadata
    latest_state_.header.source_monotonic_ns = now_ns;
    latest_state_.header.publish_monotonic_ns = now_ns;
    latest_state_.header.source_id = config_.source_id;
    latest_state_.header.validity_flags =
        protocol::kValidityPayloadValid |
        protocol::kValidityFromBackend |
        protocol::kValidityPartial;
    latest_state_.header.fault_flags = 0;

    // Mark DDS as OK
    latest_state_.dds_ok = 1;

    // Update state stamp
    state_stamp_ = std::chrono::steady_clock::now();

    // Mark lowstate as seen and increment sequence
    if (!lowstate_seen_.load(std::memory_order_acquire))
    {
        lowstate_seen_.store(true, std::memory_order_release);
    }

    // Increment lowstate sequence and stamp it onto the typed frame
    latest_state_.header.seq = IncrementLowstateSeq();
}

// ============================================================================
// Internal Helpers
// ============================================================================

void UnitreeAdapter::InitializeLowCmd()
{
    if (!lowcmd_buffer_)
    {
        return;
    }

    auto& cmd = *lowcmd_buffer_;
    cmd.head()[0] = kLowCmdHeader0;
    cmd.head()[1] = kLowCmdHeader1;
    cmd.level_flag() = kLowCmdLevelFlag;
    cmd.gpio() = 0;

    for (int i = 0; i < kUnitreeMotorCount; ++i)
    {
        cmd.motor_cmd()[i].mode() = kMotorModePseudoContinuous;
        cmd.motor_cmd()[i].q() = kPosStopF;
        cmd.motor_cmd()[i].kp() = 0.0f;
        cmd.motor_cmd()[i].dq() = kVelStopF;
        cmd.motor_cmd()[i].kd() = 0.0f;
        cmd.motor_cmd()[i].tau() = 0.0f;
    }
}

void UnitreeAdapter::ConvertBodyCommandToLowCmd(const protocol::BodyCommandFrame& body_cmd)
{
    if (!lowcmd_buffer_)
    {
        return;
    }

    auto& cmd = *lowcmd_buffer_;

    // Keep header and GPIO set from initialization
    // motor commands are updated based on body_cmd

    for (int i = 0; i < kUnitreeMotorCount; ++i)
    {
        cmd.motor_cmd()[i].mode() = kMotorModeDisabled;
        cmd.motor_cmd()[i].q() = kPosStopF;
        cmd.motor_cmd()[i].dq() = kVelStopF;
        cmd.motor_cmd()[i].kp() = 0.0f;
        cmd.motor_cmd()[i].kd() = 0.0f;
        cmd.motor_cmd()[i].tau() = 0.0f;
    }

    const int leg_count = std::min(
        static_cast<int>(body_cmd.joint_count),
        static_cast<int>(protocol::kDogJointCount));
    for (int i = 0; i < leg_count; ++i)
    {
        const int motor_index = ResolveMotorIndexForBodyJoint(i);
        if (motor_index < 0 || motor_index >= kUnitreeMotorCount)
        {
            continue;
        }
        auto& motor_cmd = cmd.motor_cmd()[motor_index];
        if (IsPassiveBodyJointCommand(body_cmd.q[static_cast<size_t>(i)],
                                      body_cmd.dq[static_cast<size_t>(i)],
                                      body_cmd.kp[static_cast<size_t>(i)],
                                      body_cmd.kd[static_cast<size_t>(i)],
                                      body_cmd.tau[static_cast<size_t>(i)]))
        {
            motor_cmd.mode() = kMotorModeDisabled;
            motor_cmd.q() = kPosStopF;
            motor_cmd.dq() = kVelStopF;
            motor_cmd.kp() = 0.0f;
            motor_cmd.kd() = 0.0f;
            motor_cmd.tau() = 0.0f;
            continue;
        }

        const bool has_valid_cmd =
            std::isfinite(body_cmd.q[static_cast<size_t>(i)]) ||
            std::isfinite(body_cmd.tau[static_cast<size_t>(i)]);

        if (has_valid_cmd)
        {
            motor_cmd.mode() = kMotorModePseudoContinuous;

            // Use feedforward torque if position is not finite
            if (std::isfinite(body_cmd.q[static_cast<size_t>(i)]))
            {
                motor_cmd.q() = body_cmd.q[static_cast<size_t>(i)];
                motor_cmd.dq() = std::isfinite(body_cmd.dq[static_cast<size_t>(i)])
                    ? body_cmd.dq[static_cast<size_t>(i)]
                    : 0.0f;
                motor_cmd.kp() = std::isfinite(body_cmd.kp[static_cast<size_t>(i)])
                    ? body_cmd.kp[static_cast<size_t>(i)]
                    : 0.0f;
                motor_cmd.kd() = std::isfinite(body_cmd.kd[static_cast<size_t>(i)])
                    ? body_cmd.kd[static_cast<size_t>(i)]
                    : 0.0f;
                motor_cmd.tau() = std::isfinite(body_cmd.tau[static_cast<size_t>(i)])
                    ? body_cmd.tau[static_cast<size_t>(i)]
                    : 0.0f;
            }
            else
            {
                // Torque-only mode
                motor_cmd.q() = kPosStopF;
                motor_cmd.dq() = kVelStopF;
                motor_cmd.kp() = 0.0f;
                motor_cmd.kd() = 0.0f;
                motor_cmd.tau() = std::isfinite(body_cmd.tau[static_cast<size_t>(i)])
                    ? body_cmd.tau[static_cast<size_t>(i)]
                    : 0.0f;
            }
        }
    }

    cmd.crc() = ComputeLowCmdCrc(cmd);
}

uint64_t UnitreeAdapter::IncrementLowstateSeq()
{
    return lowstate_seq_.fetch_add(1, std::memory_order_release) + 1;
}

uint64_t UnitreeAdapter::GetMonotonicNs()
{
    const auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

int UnitreeAdapter::ResolveMotorIndexForBodyJoint(const int body_joint_index) const
{
    if (body_joint_index < 0 ||
        body_joint_index >= static_cast<int>(config_.joint_mapping.size()))
    {
        return -1;
    }
    return config_.joint_mapping[static_cast<size_t>(body_joint_index)];
}

uint32_t UnitreeAdapter::ComputeLowCmdCrc(const unitree_go::msg::dds_::LowCmd_& cmd)
{
    return Crc32Core(
        reinterpret_cast<const uint32_t*>(&cmd),
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
}

// ============================================================================
// Utility Functions
// ============================================================================

const char* ToString(UnitreeAdapter::Status status)
{
    switch (status)
    {
        case UnitreeAdapter::Status::kOk:                     return "Ok";
        case UnitreeAdapter::Status::kNotInitialized:         return "NotInitialized";
        case UnitreeAdapter::Status::kAlreadyInitialized:     return "AlreadyInitialized";
        case UnitreeAdapter::Status::kDdsInitFailed:          return "DdsInitFailed";
        case UnitreeAdapter::Status::kPublisherCreateFailed:  return "PublisherCreateFailed";
        case UnitreeAdapter::Status::kSubscriberCreateFailed: return "SubscriberCreateFailed";
        case UnitreeAdapter::Status::kInvalidConfig:          return "InvalidConfig";
        case UnitreeAdapter::Status::kStateStale:             return "StateStale";
        case UnitreeAdapter::Status::kWriteFailed:            return "WriteFailed";
        case UnitreeAdapter::Status::kNotStarted:             return "NotStarted";
        case UnitreeAdapter::Status::kAlreadyStarted:         return "AlreadyStarted";
        default:                                              return "Unknown";
    }
}

}  // namespace rl_sar::adapters
