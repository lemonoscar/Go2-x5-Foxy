#include "rl_sar/protocol/go2_x5_protocol.hpp"

#include <cstring>
#include <type_traits>

namespace rl_sar::protocol
{
namespace
{

template <typename T>
constexpr bool IsIntegralOrEnumV = std::is_integral<T>::value || std::is_enum<T>::value;

template <typename T>
using EnableIfIntegralOrEnum = std::enable_if_t<IsIntegralOrEnumV<T>, int>;

template <typename T>
using EnableIfFloat = std::enable_if_t<std::is_floating_point<T>::value, int>;

template <typename T, bool IsEnum = std::is_enum<T>::value>
struct UnderlyingOrSelf
{
    using type = T;
};

template <typename T>
struct UnderlyingOrSelf<T, true>
{
    using type = std::underlying_type_t<T>;
};

template <typename UInt>
void AppendUnsignedLE(std::vector<uint8_t>& bytes, UInt value)
{
    for (size_t i = 0; i < sizeof(UInt); ++i)
    {
        bytes.push_back(static_cast<uint8_t>((value >> (8u * i)) & 0xFFu));
    }
}

template <typename UInt>
bool ReadUnsignedLE(const std::vector<uint8_t>& bytes, size_t& offset, UInt& value)
{
    if (offset + sizeof(UInt) > bytes.size())
    {
        return false;
    }

    UInt result = 0;
    for (size_t i = 0; i < sizeof(UInt); ++i)
    {
        result |= static_cast<UInt>(bytes[offset + i]) << (8u * i);
    }
    offset += sizeof(UInt);
    value = result;
    return true;
}

template <typename T, EnableIfIntegralOrEnum<T> = 0>
void AppendScalarLE(std::vector<uint8_t>& bytes, T value)
{
    using RawT = typename UnderlyingOrSelf<T>::type;
    using Unsigned = std::make_unsigned_t<RawT>;
    AppendUnsignedLE(bytes, static_cast<Unsigned>(value));
}

template <typename T, EnableIfFloat<T> = 0>
void AppendScalarLE(std::vector<uint8_t>& bytes, T value)
{
    using Unsigned = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
    static_assert(sizeof(T) == sizeof(Unsigned), "unexpected float size");
    Unsigned raw = 0;
    std::memcpy(&raw, &value, sizeof(T));
    AppendUnsignedLE(bytes, raw);
}

template <typename T, EnableIfIntegralOrEnum<T> = 0>
bool ReadScalarLE(const std::vector<uint8_t>& bytes, size_t& offset, T& value)
{
    using RawT = typename UnderlyingOrSelf<T>::type;
    using Unsigned = std::make_unsigned_t<RawT>;
    Unsigned raw = 0;
    if (!ReadUnsignedLE(bytes, offset, raw))
    {
        return false;
    }
    value = static_cast<T>(raw);
    return true;
}

template <typename T, EnableIfFloat<T> = 0>
bool ReadScalarLE(const std::vector<uint8_t>& bytes, size_t& offset, T& value)
{
    using Unsigned = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
    static_assert(sizeof(T) == sizeof(Unsigned), "unexpected float size");
    Unsigned raw = 0;
    if (!ReadUnsignedLE(bytes, offset, raw))
    {
        return false;
    }
    std::memcpy(&value, &raw, sizeof(T));
    return true;
}

template <typename T, size_t N>
void AppendArrayLE(std::vector<uint8_t>& bytes, const std::array<T, N>& values)
{
    for (const auto& value : values)
    {
        AppendScalarLE(bytes, value);
    }
}

template <typename T, size_t N>
bool ReadArrayLE(const std::vector<uint8_t>& bytes, size_t& offset, std::array<T, N>& values)
{
    for (auto& value : values)
    {
        if (!ReadScalarLE(bytes, offset, value))
        {
            return false;
        }
    }
    return true;
}

FrameHeader MakeHeader(const FrameHeader& input, FrameType type, uint32_t payload_bytes)
{
    FrameHeader header = input;
    header.magic = kFrameMagic;
    header.version = kProtocolVersion;
    header.msg_type = type;
    header.payload_bytes = payload_bytes;
    return header;
}

bool ParseHeaderCommon(
    const std::vector<uint8_t>& bytes,
    FrameHeader& header,
    FrameType expected_type,
    uint32_t expected_payload_bytes,
    std::string* error)
{
    if (bytes.size() != kFrameHeaderSize + static_cast<size_t>(expected_payload_bytes))
    {
        if (error)
        {
            *error = "unexpected frame size";
        }
        return false;
    }

    size_t offset = 0;
    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t msg_type = 0;
    if (!ReadScalarLE(bytes, offset, magic) ||
        !ReadScalarLE(bytes, offset, version) ||
        !ReadScalarLE(bytes, offset, msg_type) ||
        !ReadScalarLE(bytes, offset, header.seq) ||
        !ReadScalarLE(bytes, offset, header.source_monotonic_ns) ||
        !ReadScalarLE(bytes, offset, header.publish_monotonic_ns) ||
        !ReadScalarLE(bytes, offset, header.source_id) ||
        !ReadScalarLE(bytes, offset, header.mode) ||
        !ReadScalarLE(bytes, offset, header.validity_flags) ||
        !ReadScalarLE(bytes, offset, header.fault_flags) ||
        !ReadScalarLE(bytes, offset, header.payload_bytes))
    {
        if (error)
        {
            *error = "failed to read frame header";
        }
        return false;
    }

    header.magic = magic;
    header.version = version;
    header.msg_type = static_cast<FrameType>(msg_type);

    if (magic != kFrameMagic)
    {
        if (error)
        {
            *error = "invalid frame magic";
        }
        return false;
    }
    if (version != kProtocolVersion)
    {
        if (error)
        {
            *error = "unsupported frame version";
        }
        return false;
    }
    if (header.msg_type != expected_type)
    {
        if (error)
        {
            *error = "unexpected frame type";
        }
        return false;
    }
    if (header.payload_bytes != expected_payload_bytes)
    {
        if (error)
        {
            *error = "unexpected payload size";
        }
        return false;
    }
    return true;
}

std::vector<uint8_t> SerializeHeaderCommon(const FrameHeader& header)
{
    std::vector<uint8_t> bytes;
    bytes.reserve(kFrameHeaderSize);
    AppendScalarLE(bytes, header.magic);
    AppendScalarLE(bytes, header.version);
    AppendScalarLE(bytes, static_cast<uint16_t>(header.msg_type));
    AppendScalarLE(bytes, header.seq);
    AppendScalarLE(bytes, header.source_monotonic_ns);
    AppendScalarLE(bytes, header.publish_monotonic_ns);
    AppendScalarLE(bytes, header.source_id);
    AppendScalarLE(bytes, header.mode);
    AppendScalarLE(bytes, header.validity_flags);
    AppendScalarLE(bytes, header.fault_flags);
    AppendScalarLE(bytes, header.payload_bytes);
    return bytes;
}

}  // namespace

std::vector<uint8_t> SerializeFrameHeader(const FrameHeader& header)
{
    return SerializeHeaderCommon(header);
}

bool ParseFrameHeader(const std::vector<uint8_t>& bytes, FrameHeader& header, std::string* error)
{
    if (bytes.size() != kFrameHeaderSize)
    {
        if (error)
        {
            *error = "unexpected header size";
        }
        return false;
    }

    size_t offset = 0;
    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t msg_type = 0;
    if (!ReadScalarLE(bytes, offset, magic) ||
        !ReadScalarLE(bytes, offset, version) ||
        !ReadScalarLE(bytes, offset, msg_type) ||
        !ReadScalarLE(bytes, offset, header.seq) ||
        !ReadScalarLE(bytes, offset, header.source_monotonic_ns) ||
        !ReadScalarLE(bytes, offset, header.publish_monotonic_ns) ||
        !ReadScalarLE(bytes, offset, header.source_id) ||
        !ReadScalarLE(bytes, offset, header.mode) ||
        !ReadScalarLE(bytes, offset, header.validity_flags) ||
        !ReadScalarLE(bytes, offset, header.fault_flags) ||
        !ReadScalarLE(bytes, offset, header.payload_bytes))
    {
        if (error)
        {
            *error = "failed to read header";
        }
        return false;
    }

    header.magic = magic;
    header.version = version;
    header.msg_type = static_cast<FrameType>(msg_type);
    if (magic != kFrameMagic)
    {
        if (error)
        {
            *error = "invalid frame magic";
        }
        return false;
    }
    if (version != kProtocolVersion)
    {
        if (error)
        {
            *error = "unsupported frame version";
        }
        return false;
    }
    return true;
}

std::vector<uint8_t> SerializeBodyStateFrame(const BodyStateFrame& frame)
{
    BodyStateFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::BodyState, static_cast<uint32_t>(kBodyStatePayloadSize));

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendArrayLE(bytes, copy.imu_quat);
    AppendArrayLE(bytes, copy.imu_gyro);
    AppendArrayLE(bytes, copy.imu_acc);
    AppendArrayLE(bytes, copy.leg_q);
    AppendArrayLE(bytes, copy.leg_dq);
    AppendArrayLE(bytes, copy.leg_tau);
    AppendArrayLE(bytes, copy.base_lin_vel);
    AppendArrayLE(bytes, copy.base_ang_vel);
    AppendArrayLE(bytes, copy.projected_gravity);
    AppendScalarLE(bytes, copy.lowstate_age_us);
    AppendScalarLE(bytes, copy.dds_ok);
    AppendArrayLE(bytes, copy.reserved);
    return bytes;
}

bool ParseBodyStateFrame(const std::vector<uint8_t>& bytes, BodyStateFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::BodyState, static_cast<uint32_t>(kBodyStatePayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadArrayLE(bytes, offset, frame.imu_quat) &&
           ReadArrayLE(bytes, offset, frame.imu_gyro) &&
           ReadArrayLE(bytes, offset, frame.imu_acc) &&
           ReadArrayLE(bytes, offset, frame.leg_q) &&
           ReadArrayLE(bytes, offset, frame.leg_dq) &&
           ReadArrayLE(bytes, offset, frame.leg_tau) &&
           ReadArrayLE(bytes, offset, frame.base_lin_vel) &&
           ReadArrayLE(bytes, offset, frame.base_ang_vel) &&
           ReadArrayLE(bytes, offset, frame.projected_gravity) &&
           ReadScalarLE(bytes, offset, frame.lowstate_age_us) &&
           ReadScalarLE(bytes, offset, frame.dds_ok) &&
           ReadArrayLE(bytes, offset, frame.reserved);
}

std::vector<uint8_t> SerializeArmStateFrame(const ArmStateFrame& frame)
{
    ArmStateFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::ArmState, static_cast<uint32_t>(kArmStatePayloadSize));

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.joint_count);
    AppendScalarLE(bytes, copy.backend_mode);
    AppendArrayLE(bytes, copy.q);
    AppendArrayLE(bytes, copy.dq);
    AppendArrayLE(bytes, copy.tau);
    AppendArrayLE(bytes, copy.q_target);
    AppendArrayLE(bytes, copy.tracking_error);
    AppendScalarLE(bytes, copy.backend_age_us);
    AppendScalarLE(bytes, copy.transport_age_us);
    AppendScalarLE(bytes, copy.target_seq_applied);
    return bytes;
}

bool ParseArmStateFrame(const std::vector<uint8_t>& bytes, ArmStateFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::ArmState, static_cast<uint32_t>(kArmStatePayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.joint_count) &&
           ReadScalarLE(bytes, offset, frame.backend_mode) &&
           ReadArrayLE(bytes, offset, frame.q) &&
           ReadArrayLE(bytes, offset, frame.dq) &&
           ReadArrayLE(bytes, offset, frame.tau) &&
           ReadArrayLE(bytes, offset, frame.q_target) &&
           ReadArrayLE(bytes, offset, frame.tracking_error) &&
           ReadScalarLE(bytes, offset, frame.backend_age_us) &&
           ReadScalarLE(bytes, offset, frame.transport_age_us) &&
           ReadScalarLE(bytes, offset, frame.target_seq_applied);
}

std::vector<uint8_t> SerializeOperatorCommandFrame(const OperatorCommandFrame& frame)
{
    OperatorCommandFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::OperatorCommand, static_cast<uint32_t>(kOperatorCommandPayloadSize));

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.cmd_source);
    AppendScalarLE(bytes, copy.key_mask);
    AppendScalarLE(bytes, copy.e_stop);
    AppendScalarLE(bytes, copy.locomotion_enable);
    AppendScalarLE(bytes, copy.arm_enable);
    AppendScalarLE(bytes, copy.reserved0);
    AppendScalarLE(bytes, copy.cmd_vel_x);
    AppendScalarLE(bytes, copy.cmd_vel_y);
    AppendScalarLE(bytes, copy.cmd_vel_yaw);
    AppendArrayLE(bytes, copy.arm_target_q);
    AppendScalarLE(bytes, copy.gripper_target);
    return bytes;
}

bool ParseOperatorCommandFrame(const std::vector<uint8_t>& bytes, OperatorCommandFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::OperatorCommand, static_cast<uint32_t>(kOperatorCommandPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.cmd_source) &&
           ReadScalarLE(bytes, offset, frame.key_mask) &&
           ReadScalarLE(bytes, offset, frame.e_stop) &&
           ReadScalarLE(bytes, offset, frame.locomotion_enable) &&
           ReadScalarLE(bytes, offset, frame.arm_enable) &&
           ReadScalarLE(bytes, offset, frame.reserved0) &&
           ReadScalarLE(bytes, offset, frame.cmd_vel_x) &&
           ReadScalarLE(bytes, offset, frame.cmd_vel_y) &&
           ReadScalarLE(bytes, offset, frame.cmd_vel_yaw) &&
           ReadArrayLE(bytes, offset, frame.arm_target_q) &&
           ReadScalarLE(bytes, offset, frame.gripper_target);
}

std::vector<uint8_t> SerializeDogPolicyObservationFrame(const DogPolicyObservationFrame& frame)
{
    DogPolicyObservationFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::DogPolicyObservation, static_cast<uint32_t>(kDogPolicyObservationPayloadSize));

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendArrayLE(bytes, copy.base_lin_vel);
    AppendArrayLE(bytes, copy.base_ang_vel);
    AppendArrayLE(bytes, copy.projected_gravity);
    AppendArrayLE(bytes, copy.velocity_commands);
    AppendArrayLE(bytes, copy.full_joint_pos);
    AppendArrayLE(bytes, copy.full_joint_vel);
    AppendArrayLE(bytes, copy.last_actions);
    AppendArrayLE(bytes, copy.height_scan);
    AppendArrayLE(bytes, copy.arm_joint_command);
    AppendArrayLE(bytes, copy.gripper_command);
    return bytes;
}

bool ParseDogPolicyObservationFrame(const std::vector<uint8_t>& bytes, DogPolicyObservationFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::DogPolicyObservation, static_cast<uint32_t>(kDogPolicyObservationPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadArrayLE(bytes, offset, frame.base_lin_vel) &&
           ReadArrayLE(bytes, offset, frame.base_ang_vel) &&
           ReadArrayLE(bytes, offset, frame.projected_gravity) &&
           ReadArrayLE(bytes, offset, frame.velocity_commands) &&
           ReadArrayLE(bytes, offset, frame.full_joint_pos) &&
           ReadArrayLE(bytes, offset, frame.full_joint_vel) &&
           ReadArrayLE(bytes, offset, frame.last_actions) &&
           ReadArrayLE(bytes, offset, frame.height_scan) &&
           ReadArrayLE(bytes, offset, frame.arm_joint_command) &&
           ReadArrayLE(bytes, offset, frame.gripper_command);
}

std::vector<uint8_t> SerializeDogPolicyCommandFrame(const DogPolicyCommandFrame& frame)
{
    DogPolicyCommandFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::DogPolicyCommand, static_cast<uint32_t>(kDogPolicyCommandPayloadSize));
    copy.action_dim = kDogJointCount;

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.policy_id_hash);
    AppendScalarLE(bytes, copy.action_dim);
    AppendScalarLE(bytes, copy.reserved0);
    AppendScalarLE(bytes, copy.inference_latency_us);
    AppendArrayLE(bytes, copy.leg_action);
    return bytes;
}

bool ParseDogPolicyCommandFrame(const std::vector<uint8_t>& bytes, DogPolicyCommandFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::DogPolicyCommand, static_cast<uint32_t>(kDogPolicyCommandPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.policy_id_hash) &&
           ReadScalarLE(bytes, offset, frame.action_dim) &&
           ReadScalarLE(bytes, offset, frame.reserved0) &&
           ReadScalarLE(bytes, offset, frame.inference_latency_us) &&
           ReadArrayLE(bytes, offset, frame.leg_action);
}

std::vector<uint8_t> SerializeArmCommandFrame(const ArmCommandFrame& frame)
{
    ArmCommandFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::ArmCommand, static_cast<uint32_t>(kArmCommandPayloadSize));
    copy.joint_count = kArmJointCount;

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.joint_count);
    AppendScalarLE(bytes, copy.interpolation_hint);
    AppendScalarLE(bytes, copy.command_expire_ns);
    AppendArrayLE(bytes, copy.q);
    AppendArrayLE(bytes, copy.dq);
    AppendArrayLE(bytes, copy.kp);
    AppendArrayLE(bytes, copy.kd);
    AppendArrayLE(bytes, copy.tau);
    AppendScalarLE(bytes, copy.gripper_target);
    return bytes;
}

bool ParseArmCommandFrame(const std::vector<uint8_t>& bytes, ArmCommandFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::ArmCommand, static_cast<uint32_t>(kArmCommandPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.joint_count) &&
           ReadScalarLE(bytes, offset, frame.interpolation_hint) &&
           ReadScalarLE(bytes, offset, frame.command_expire_ns) &&
           ReadArrayLE(bytes, offset, frame.q) &&
           ReadArrayLE(bytes, offset, frame.dq) &&
           ReadArrayLE(bytes, offset, frame.kp) &&
           ReadArrayLE(bytes, offset, frame.kd) &&
           ReadArrayLE(bytes, offset, frame.tau) &&
           ReadScalarLE(bytes, offset, frame.gripper_target);
}

std::vector<uint8_t> SerializeBodyCommandFrame(const BodyCommandFrame& frame)
{
    BodyCommandFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::BodyCommand, static_cast<uint32_t>(kBodyCommandPayloadSize));
    copy.joint_count = kBodyJointCount;

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.joint_count);
    AppendScalarLE(bytes, copy.reserved0);
    AppendScalarLE(bytes, copy.command_expire_ns);
    AppendArrayLE(bytes, copy.q);
    AppendArrayLE(bytes, copy.dq);
    AppendArrayLE(bytes, copy.kp);
    AppendArrayLE(bytes, copy.kd);
    AppendArrayLE(bytes, copy.tau);
    return bytes;
}

bool ParseBodyCommandFrame(const std::vector<uint8_t>& bytes, BodyCommandFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::BodyCommand, static_cast<uint32_t>(kBodyCommandPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.joint_count) &&
           ReadScalarLE(bytes, offset, frame.reserved0) &&
           ReadScalarLE(bytes, offset, frame.command_expire_ns) &&
           ReadArrayLE(bytes, offset, frame.q) &&
           ReadArrayLE(bytes, offset, frame.dq) &&
           ReadArrayLE(bytes, offset, frame.kp) &&
           ReadArrayLE(bytes, offset, frame.kd) &&
           ReadArrayLE(bytes, offset, frame.tau);
}

std::vector<uint8_t> SerializeHybridDiagnosticFrame(const HybridDiagnosticFrame& frame)
{
    HybridDiagnosticFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::HybridDiagnostic, static_cast<uint32_t>(kHybridDiagnosticPayloadSize));

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.body_state_age_us);
    AppendScalarLE(bytes, copy.arm_state_age_us);
    AppendScalarLE(bytes, copy.policy_latency_us);
    AppendScalarLE(bytes, copy.coordinator_jitter_us);
    AppendScalarLE(bytes, copy.arm_tracking_error_norm);
    AppendScalarLE(bytes, copy.xy_drift_error);
    AppendScalarLE(bytes, copy.yaw_drift_error);
    AppendScalarLE(bytes, copy.clip_count);
    AppendScalarLE(bytes, copy.dds_write_fail_count);
    AppendScalarLE(bytes, copy.arm_backend_fail_count);
    AppendScalarLE(bytes, copy.seq_gap_count);
    return bytes;
}

bool ParseHybridDiagnosticFrame(const std::vector<uint8_t>& bytes, HybridDiagnosticFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::HybridDiagnostic, static_cast<uint32_t>(kHybridDiagnosticPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.body_state_age_us) &&
           ReadScalarLE(bytes, offset, frame.arm_state_age_us) &&
           ReadScalarLE(bytes, offset, frame.policy_latency_us) &&
           ReadScalarLE(bytes, offset, frame.coordinator_jitter_us) &&
           ReadScalarLE(bytes, offset, frame.arm_tracking_error_norm) &&
           ReadScalarLE(bytes, offset, frame.xy_drift_error) &&
           ReadScalarLE(bytes, offset, frame.yaw_drift_error) &&
           ReadScalarLE(bytes, offset, frame.clip_count) &&
           ReadScalarLE(bytes, offset, frame.dds_write_fail_count) &&
           ReadScalarLE(bytes, offset, frame.arm_backend_fail_count) &&
           ReadScalarLE(bytes, offset, frame.seq_gap_count);
}

std::vector<uint8_t> SerializeModeEventFrame(const ModeEventFrame& frame)
{
    ModeEventFrame copy = frame;
    copy.header = MakeHeader(frame.header, FrameType::ModeEvent, static_cast<uint32_t>(kModeEventPayloadSize));

    std::vector<uint8_t> bytes = SerializeHeaderCommon(copy.header);
    AppendScalarLE(bytes, copy.from_mode);
    AppendScalarLE(bytes, copy.to_mode);
    AppendScalarLE(bytes, copy.reason_code);
    AppendScalarLE(bytes, copy.trigger_seq);
    AppendScalarLE(bytes, copy.detail_value);
    return bytes;
}

bool ParseModeEventFrame(const std::vector<uint8_t>& bytes, ModeEventFrame& frame, std::string* error)
{
    if (!ParseHeaderCommon(bytes, frame.header, FrameType::ModeEvent, static_cast<uint32_t>(kModeEventPayloadSize), error))
    {
        return false;
    }

    size_t offset = kFrameHeaderSize;
    return ReadScalarLE(bytes, offset, frame.from_mode) &&
           ReadScalarLE(bytes, offset, frame.to_mode) &&
           ReadScalarLE(bytes, offset, frame.reason_code) &&
           ReadScalarLE(bytes, offset, frame.trigger_seq) &&
           ReadScalarLE(bytes, offset, frame.detail_value);
}

}  // namespace rl_sar::protocol
