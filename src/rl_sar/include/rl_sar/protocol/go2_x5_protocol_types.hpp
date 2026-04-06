#ifndef RL_SAR_PROTOCOL_GO2_X5_PROTOCOL_TYPES_HPP
#define RL_SAR_PROTOCOL_GO2_X5_PROTOCOL_TYPES_HPP

#include <array>
#include <cstdint>

namespace rl_sar::protocol
{

constexpr uint32_t kFrameMagic = 0x50355847u;  // "GX5P" in little-endian wire order
constexpr uint16_t kProtocolVersion = 1u;

constexpr uint16_t kDogJointCount = 12u;
constexpr uint16_t kArmJointCount = 6u;
constexpr uint16_t kBodyJointCount = 12u;

enum class FrameType : uint16_t
{
    BodyState = 1,
    ArmState = 2,
    OperatorCommand = 3,
    DogPolicyObservation = 4,
    DogPolicyCommand = 5,
    ArmCommand = 6,
    BodyCommand = 7,
    HybridDiagnostic = 8,
    ModeEvent = 9,
};

constexpr uint16_t kValidityPayloadValid = 1u << 0;
constexpr uint16_t kValidityFromBackend = 1u << 1;
constexpr uint16_t kValidityStale = 1u << 2;
constexpr uint16_t kValidityShadowState = 1u << 3;
constexpr uint16_t kValidityClipped = 1u << 4;
constexpr uint16_t kValidityExpired = 1u << 5;
constexpr uint16_t kValidityPartial = 1u << 6;
constexpr uint16_t kValidityFallbackGenerated = 1u << 7;

constexpr uint32_t kFaultTimeout = 1u << 0;
constexpr uint32_t kFaultSequenceGap = 1u << 1;
constexpr uint32_t kFaultTransportError = 1u << 2;
constexpr uint32_t kFaultBackendInvalid = 1u << 3;
constexpr uint32_t kFaultSizeMismatch = 1u << 4;
constexpr uint32_t kFaultIllegalJointCount = 1u << 5;
constexpr uint32_t kFaultSafetyReject = 1u << 6;
constexpr uint32_t kFaultModeReject = 1u << 7;

constexpr size_t kFrameHeaderSize = 48u;
constexpr size_t kBodyStatePayloadSize = 228u;
constexpr size_t kArmStatePayloadSize = 140u;
constexpr size_t kOperatorCommandPayloadSize = 48u;
constexpr size_t kDogPolicyObservationPayloadSize = 1040u;
constexpr size_t kDogPolicyCommandPayloadSize = 64u;
constexpr size_t kArmCommandPayloadSize = 136u;
constexpr size_t kBodyCommandPayloadSize = 252u;
constexpr size_t kHybridDiagnosticPayloadSize = 44u;
constexpr size_t kModeEventPayloadSize = 24u;

struct FrameHeader
{
    uint32_t magic = kFrameMagic;
    uint16_t version = kProtocolVersion;
    FrameType msg_type = FrameType::BodyState;
    uint64_t seq = 0;
    uint64_t source_monotonic_ns = 0;
    uint64_t publish_monotonic_ns = 0;
    uint32_t source_id = 0;
    uint16_t mode = 0;
    uint16_t validity_flags = 0;
    uint32_t fault_flags = 0;
    uint32_t payload_bytes = 0;
};

struct BodyStateFrame
{
    FrameHeader header;
    std::array<float, 4> imu_quat{};
    std::array<float, 3> imu_gyro{};
    std::array<float, 3> imu_acc{};
    std::array<float, 12> leg_q{};
    std::array<float, 12> leg_dq{};
    std::array<float, 12> leg_tau{};
    std::array<float, 3> base_lin_vel{};
    std::array<float, 3> base_ang_vel{};
    std::array<float, 3> projected_gravity{};
    uint32_t lowstate_age_us = 0;
    uint8_t dds_ok = 0;
    std::array<uint8_t, 3> reserved{};

    BodyStateFrame()
    {
        header.msg_type = FrameType::BodyState;
        header.payload_bytes = static_cast<uint32_t>(kBodyStatePayloadSize);
    }
};

struct ArmStateFrame
{
    FrameHeader header;
    uint16_t joint_count = kArmJointCount;
    uint16_t backend_mode = 0;
    std::array<float, 6> q{};
    std::array<float, 6> dq{};
    std::array<float, 6> tau{};
    std::array<float, 6> q_target{};
    std::array<float, 6> tracking_error{};
    uint32_t backend_age_us = 0;
    uint32_t transport_age_us = 0;
    uint64_t target_seq_applied = 0;

    ArmStateFrame()
    {
        header.msg_type = FrameType::ArmState;
        header.payload_bytes = static_cast<uint32_t>(kArmStatePayloadSize);
    }
};

struct OperatorCommandFrame
{
    FrameHeader header;
    uint16_t cmd_source = 0;
    uint16_t key_mask = 0;
    uint8_t e_stop = 0;
    uint8_t locomotion_enable = 0;
    uint8_t arm_enable = 0;
    uint8_t reserved0 = 0;
    float cmd_vel_x = 0.0f;
    float cmd_vel_y = 0.0f;
    float cmd_vel_yaw = 0.0f;
    std::array<float, 6> arm_target_q{};
    float gripper_target = 0.0f;

    OperatorCommandFrame()
    {
        header.msg_type = FrameType::OperatorCommand;
        header.payload_bytes = static_cast<uint32_t>(kOperatorCommandPayloadSize);
    }
};

struct DogPolicyObservationFrame
{
    FrameHeader header;
    std::array<float, 3> base_lin_vel{};
    std::array<float, 3> base_ang_vel{};
    std::array<float, 3> projected_gravity{};
    std::array<float, 3> velocity_commands{};
    std::array<float, 18> full_joint_pos{};
    std::array<float, 18> full_joint_vel{};
    std::array<float, 18> last_actions{};
    std::array<float, 187> height_scan{};
    std::array<float, 6> arm_joint_command{};
    std::array<float, 1> gripper_command{};

    DogPolicyObservationFrame()
    {
        header.msg_type = FrameType::DogPolicyObservation;
        header.payload_bytes = static_cast<uint32_t>(kDogPolicyObservationPayloadSize);
    }
};

struct DogPolicyCommandFrame
{
    FrameHeader header;
    uint64_t policy_id_hash = 0;
    uint16_t action_dim = kDogJointCount;
    uint16_t reserved0 = 0;
    uint32_t inference_latency_us = 0;
    std::array<float, 12> leg_action{};

    DogPolicyCommandFrame()
    {
        header.msg_type = FrameType::DogPolicyCommand;
        header.payload_bytes = static_cast<uint32_t>(kDogPolicyCommandPayloadSize);
    }
};

struct ArmCommandFrame
{
    FrameHeader header;
    uint16_t joint_count = kArmJointCount;
    uint16_t interpolation_hint = 0;
    uint64_t command_expire_ns = 0;
    std::array<float, 6> q{};
    std::array<float, 6> dq{};
    std::array<float, 6> kp{};
    std::array<float, 6> kd{};
    std::array<float, 6> tau{};
    float gripper_target = 0.0f;

    ArmCommandFrame()
    {
        header.msg_type = FrameType::ArmCommand;
        header.payload_bytes = static_cast<uint32_t>(kArmCommandPayloadSize);
    }
};

struct BodyCommandFrame
{
    FrameHeader header;
    uint16_t joint_count = kBodyJointCount;
    uint16_t reserved0 = 0;
    uint64_t command_expire_ns = 0;
    std::array<float, 12> q{};
    std::array<float, 12> dq{};
    std::array<float, 12> kp{};
    std::array<float, 12> kd{};
    std::array<float, 12> tau{};

    BodyCommandFrame()
    {
        header.msg_type = FrameType::BodyCommand;
        header.payload_bytes = static_cast<uint32_t>(kBodyCommandPayloadSize);
    }
};

struct HybridDiagnosticFrame
{
    FrameHeader header;
    uint32_t body_state_age_us = 0;
    uint32_t arm_state_age_us = 0;
    uint32_t policy_latency_us = 0;
    uint32_t coordinator_jitter_us = 0;
    float arm_tracking_error_norm = 0.0f;
    float xy_drift_error = 0.0f;
    float yaw_drift_error = 0.0f;
    uint32_t clip_count = 0;
    uint32_t dds_write_fail_count = 0;
    uint32_t arm_backend_fail_count = 0;
    uint32_t seq_gap_count = 0;

    HybridDiagnosticFrame()
    {
        header.msg_type = FrameType::HybridDiagnostic;
        header.payload_bytes = static_cast<uint32_t>(kHybridDiagnosticPayloadSize);
    }
};

struct ModeEventFrame
{
    FrameHeader header;
    uint16_t from_mode = 0;
    uint16_t to_mode = 0;
    uint32_t reason_code = 0;
    uint64_t trigger_seq = 0;
    uint64_t detail_value = 0;

    ModeEventFrame()
    {
        header.msg_type = FrameType::ModeEvent;
        header.payload_bytes = static_cast<uint32_t>(kModeEventPayloadSize);
    }
};

}  // namespace rl_sar::protocol

#endif  // RL_SAR_PROTOCOL_GO2_X5_PROTOCOL_TYPES_HPP
