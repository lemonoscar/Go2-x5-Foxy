#include <array>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <string>
#include <tuple>

#include "rl_sar/protocol/go2_x5_protocol.hpp"

namespace
{

using namespace rl_sar::protocol;

template <typename T, std::size_t N>
void FillArray(std::array<T, N>& values, T start, T step)
{
    for (std::size_t i = 0; i < N; ++i)
    {
        values[i] = static_cast<T>(start + step * static_cast<T>(i));
    }
}

template <typename T, std::size_t N>
bool ExpectArrayEq(const std::array<T, N>& lhs, const std::array<T, N>& rhs, const char* name)
{
    for (std::size_t i = 0; i < N; ++i)
    {
        if (lhs[i] != rhs[i])
        {
            std::cerr << name << " mismatch at index " << i << '\n';
            return false;
        }
    }
    return true;
}

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

void TestStaticShapes()
{
    static_assert(kDogJointCount == 12);
    static_assert(kArmJointCount == 6);
    static_assert(kBodyJointCount == 12);
    static_assert(std::tuple_size_v<decltype(BodyStateFrame{}.leg_q)> == kDogJointCount);
    static_assert(std::tuple_size_v<decltype(ArmStateFrame{}.q)> == kArmJointCount);
    static_assert(std::tuple_size_v<decltype(OperatorCommandFrame{}.arm_target_q)> == kArmJointCount);
    static_assert(std::tuple_size_v<decltype(DogPolicyCommandFrame{}.leg_action)> == kDogJointCount);
    static_assert(std::tuple_size_v<decltype(ArmCommandFrame{}.q)> == kArmJointCount);
    static_assert(std::tuple_size_v<decltype(BodyCommandFrame{}.q)> == kBodyJointCount);
}

void TestHeaderRoundTrip()
{
    FrameHeader header;
    header.msg_type = FrameType::ModeEvent;
    header.seq = 42;
    header.source_monotonic_ns = 1000;
    header.publish_monotonic_ns = 1200;
    header.source_id = 7;
    header.mode = 3;
    header.validity_flags = kValidityPayloadValid | kValidityFromBackend;
    header.fault_flags = kFaultSequenceGap;
    header.payload_bytes = 12;

    const auto bytes = SerializeFrameHeader(header);
    Require(bytes.size() == kFrameHeaderSize, "frame header size mismatch");

    FrameHeader parsed;
    std::string error;
    Require(ParseFrameHeader(bytes, parsed, &error), error.c_str());
    Require(parsed.magic == kFrameMagic, "frame header magic mismatch");
    Require(parsed.version == kProtocolVersion, "frame header version mismatch");
    Require(parsed.msg_type == FrameType::ModeEvent, "frame header msg_type mismatch");
    Require(parsed.seq == header.seq, "frame header seq mismatch");
    Require(parsed.source_monotonic_ns == header.source_monotonic_ns, "frame header source timestamp mismatch");
    Require(parsed.publish_monotonic_ns == header.publish_monotonic_ns, "frame header publish timestamp mismatch");
    Require(parsed.source_id == header.source_id, "frame header source id mismatch");
    Require(parsed.mode == header.mode, "frame header mode mismatch");
    Require(parsed.validity_flags == header.validity_flags, "frame header validity mismatch");
    Require(parsed.fault_flags == header.fault_flags, "frame header fault mismatch");
    Require(parsed.payload_bytes == header.payload_bytes, "frame header payload size mismatch");
}

void TestHelpers()
{
    Require(AgeNs(2000, 1500) == 500, "AgeNs mismatch");
    Require(IsFresh(2000, 1500, 500), "IsFresh helper should accept boundary age");
    Require(!IsFresh(2001, 1500, 500), "IsFresh helper should reject stale age");
    Require(SeqGap(10, 13) == 3, "SeqGap mismatch");
    Require(!HasSeqGap(10, 11), "HasSeqGap should accept contiguous sequence");
    Require(HasSeqGap(10, 12), "HasSeqGap should detect a gap");
    Require(!IsCommandExpired(999, 1000), "IsCommandExpired should stay false before deadline");
    Require(IsCommandExpired(1000, 1000), "IsCommandExpired should trip at deadline");

    FrameHeader header;
    header.source_monotonic_ns = 100;
    Require(IsFresh(header, 150, 50), "FrameHeader IsFresh overload mismatch");
}

void TestFrameRoundTrips()
{
    {
        BodyStateFrame input;
        input.header.msg_type = FrameType::BodyState;
        input.header.seq = 1;
        input.header.source_monotonic_ns = 1000;
        input.header.publish_monotonic_ns = 1100;
        input.header.source_id = 11;
        input.header.mode = 2;
        FillArray(input.imu_quat, 0.1f, 0.1f);
        FillArray(input.imu_gyro, 1.0f, 0.2f);
        FillArray(input.imu_acc, 2.0f, 0.3f);
        FillArray(input.leg_q, 3.0f, 0.4f);
        FillArray(input.leg_dq, 4.0f, 0.5f);
        FillArray(input.leg_tau, 5.0f, 0.6f);
        FillArray(input.base_lin_vel, 6.0f, 0.7f);
        FillArray(input.base_ang_vel, 7.0f, 0.8f);
        FillArray(input.projected_gravity, 8.0f, 0.9f);
        input.lowstate_age_us = 1234;
        input.dds_ok = 1;
        input.reserved = {9, 8, 7};

        const auto bytes = SerializeBodyStateFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kBodyStatePayloadSize, "BodyStateFrame size mismatch");

        BodyStateFrame parsed;
        std::string error;
        Require(ParseBodyStateFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::BodyState, "BodyStateFrame type mismatch");
        Require(parsed.lowstate_age_us == input.lowstate_age_us, "BodyStateFrame lowstate age mismatch");
        Require(parsed.dds_ok == input.dds_ok, "BodyStateFrame dds flag mismatch");
        Require(parsed.reserved == input.reserved, "BodyStateFrame reserved mismatch");
        Require(ExpectArrayEq(parsed.imu_quat, input.imu_quat, "imu_quat"), "BodyStateFrame imu_quat mismatch");
        Require(ExpectArrayEq(parsed.imu_gyro, input.imu_gyro, "imu_gyro"), "BodyStateFrame imu_gyro mismatch");
        Require(ExpectArrayEq(parsed.imu_acc, input.imu_acc, "imu_acc"), "BodyStateFrame imu_acc mismatch");
        Require(ExpectArrayEq(parsed.leg_q, input.leg_q, "leg_q"), "BodyStateFrame leg_q mismatch");
        Require(ExpectArrayEq(parsed.leg_dq, input.leg_dq, "leg_dq"), "BodyStateFrame leg_dq mismatch");
        Require(ExpectArrayEq(parsed.leg_tau, input.leg_tau, "leg_tau"), "BodyStateFrame leg_tau mismatch");
        Require(ExpectArrayEq(parsed.base_lin_vel, input.base_lin_vel, "base_lin_vel"), "BodyStateFrame base_lin_vel mismatch");
        Require(ExpectArrayEq(parsed.base_ang_vel, input.base_ang_vel, "base_ang_vel"), "BodyStateFrame base_ang_vel mismatch");
        Require(ExpectArrayEq(parsed.projected_gravity, input.projected_gravity, "projected_gravity"), "BodyStateFrame gravity mismatch");
    }

    {
        ArmStateFrame input;
        input.header.msg_type = FrameType::ArmState;
        input.header.seq = 2;
        input.header.source_monotonic_ns = 2000;
        input.header.publish_monotonic_ns = 2100;
        input.joint_count = kArmJointCount;
        input.backend_mode = 1;
        FillArray(input.q, 0.5f, 0.25f);
        FillArray(input.dq, 1.5f, 0.25f);
        FillArray(input.tau, 2.5f, 0.25f);
        FillArray(input.q_target, 3.5f, 0.25f);
        FillArray(input.tracking_error, 4.5f, 0.25f);
        input.backend_age_us = 100;
        input.transport_age_us = 200;
        input.target_seq_applied = 99;

        const auto bytes = SerializeArmStateFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kArmStatePayloadSize, "ArmStateFrame size mismatch");

        ArmStateFrame parsed;
        std::string error;
        Require(ParseArmStateFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::ArmState, "ArmStateFrame type mismatch");
        Require(parsed.joint_count == kArmJointCount, "ArmStateFrame joint count mismatch");
        Require(parsed.backend_mode == input.backend_mode, "ArmStateFrame backend mode mismatch");
        Require(parsed.backend_age_us == input.backend_age_us, "ArmStateFrame backend age mismatch");
        Require(parsed.transport_age_us == input.transport_age_us, "ArmStateFrame transport age mismatch");
        Require(parsed.target_seq_applied == input.target_seq_applied, "ArmStateFrame target seq mismatch");
        Require(ExpectArrayEq(parsed.q, input.q, "arm_q"), "ArmStateFrame q mismatch");
        Require(ExpectArrayEq(parsed.dq, input.dq, "arm_dq"), "ArmStateFrame dq mismatch");
        Require(ExpectArrayEq(parsed.tau, input.tau, "arm_tau"), "ArmStateFrame tau mismatch");
        Require(ExpectArrayEq(parsed.q_target, input.q_target, "arm_q_target"), "ArmStateFrame q_target mismatch");
        Require(ExpectArrayEq(parsed.tracking_error, input.tracking_error, "arm_tracking_error"), "ArmStateFrame tracking error mismatch");
    }

    {
        OperatorCommandFrame input;
        input.header.msg_type = FrameType::OperatorCommand;
        input.header.seq = 3;
        input.cmd_source = 2;
        input.key_mask = 0x55AA;
        input.e_stop = 1;
        input.locomotion_enable = 1;
        input.arm_enable = 0;
        input.cmd_vel_x = 0.5f;
        input.cmd_vel_y = -0.1f;
        input.cmd_vel_yaw = 0.25f;
        FillArray(input.arm_target_q, 0.1f, 0.2f);
        input.gripper_target = 0.33f;

        const auto bytes = SerializeOperatorCommandFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kOperatorCommandPayloadSize, "OperatorCommandFrame size mismatch");

        OperatorCommandFrame parsed;
        std::string error;
        Require(ParseOperatorCommandFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::OperatorCommand, "OperatorCommandFrame type mismatch");
        Require(parsed.cmd_source == input.cmd_source, "OperatorCommandFrame source mismatch");
        Require(parsed.key_mask == input.key_mask, "OperatorCommandFrame key mask mismatch");
        Require(parsed.e_stop == input.e_stop, "OperatorCommandFrame e_stop mismatch");
        Require(parsed.locomotion_enable == input.locomotion_enable, "OperatorCommandFrame locomotion mismatch");
        Require(parsed.arm_enable == input.arm_enable, "OperatorCommandFrame arm enable mismatch");
        Require(parsed.cmd_vel_x == input.cmd_vel_x, "OperatorCommandFrame cmd_vel_x mismatch");
        Require(parsed.cmd_vel_y == input.cmd_vel_y, "OperatorCommandFrame cmd_vel_y mismatch");
        Require(parsed.cmd_vel_yaw == input.cmd_vel_yaw, "OperatorCommandFrame cmd_vel_yaw mismatch");
        Require(parsed.gripper_target == input.gripper_target, "OperatorCommandFrame gripper mismatch");
        Require(ExpectArrayEq(parsed.arm_target_q, input.arm_target_q, "arm_target_q"), "OperatorCommandFrame arm_target_q mismatch");
    }

    {
        DogPolicyObservationFrame input;
        input.header.msg_type = FrameType::DogPolicyObservation;
        input.header.seq = 4;
        FillArray(input.base_lin_vel, 0.1f, 0.1f);
        FillArray(input.base_ang_vel, 1.0f, 0.1f);
        FillArray(input.projected_gravity, 2.0f, 0.1f);
        FillArray(input.velocity_commands, 3.0f, 0.1f);
        FillArray(input.full_joint_pos, 4.0f, 0.1f);
        FillArray(input.full_joint_vel, 5.0f, 0.1f);
        FillArray(input.last_actions, 6.0f, 0.1f);
        FillArray(input.height_scan, 7.0f, 0.1f);
        FillArray(input.arm_joint_command, 8.0f, 0.1f);
        FillArray(input.gripper_command, 9.0f, 0.1f);

        const auto bytes = SerializeDogPolicyObservationFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kDogPolicyObservationPayloadSize, "DogPolicyObservationFrame size mismatch");

        DogPolicyObservationFrame parsed;
        std::string error;
        Require(ParseDogPolicyObservationFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::DogPolicyObservation, "DogPolicyObservationFrame type mismatch");
        Require(ExpectArrayEq(parsed.base_lin_vel, input.base_lin_vel, "base_lin_vel"), "DogPolicyObservationFrame base_lin_vel mismatch");
        Require(ExpectArrayEq(parsed.base_ang_vel, input.base_ang_vel, "base_ang_vel"), "DogPolicyObservationFrame base_ang_vel mismatch");
        Require(ExpectArrayEq(parsed.projected_gravity, input.projected_gravity, "projected_gravity"), "DogPolicyObservationFrame projected gravity mismatch");
        Require(ExpectArrayEq(parsed.velocity_commands, input.velocity_commands, "velocity_commands"), "DogPolicyObservationFrame velocity_commands mismatch");
        Require(ExpectArrayEq(parsed.full_joint_pos, input.full_joint_pos, "full_joint_pos"), "DogPolicyObservationFrame full_joint_pos mismatch");
        Require(ExpectArrayEq(parsed.full_joint_vel, input.full_joint_vel, "full_joint_vel"), "DogPolicyObservationFrame full_joint_vel mismatch");
        Require(ExpectArrayEq(parsed.last_actions, input.last_actions, "last_actions"), "DogPolicyObservationFrame last_actions mismatch");
        Require(ExpectArrayEq(parsed.height_scan, input.height_scan, "height_scan"), "DogPolicyObservationFrame height_scan mismatch");
        Require(ExpectArrayEq(parsed.arm_joint_command, input.arm_joint_command, "arm_joint_command"), "DogPolicyObservationFrame arm_joint_command mismatch");
        Require(ExpectArrayEq(parsed.gripper_command, input.gripper_command, "gripper_command"), "DogPolicyObservationFrame gripper mismatch");
    }

    {
        DogPolicyCommandFrame input;
        input.header.msg_type = FrameType::DogPolicyCommand;
        input.header.seq = 5;
        input.policy_id_hash = 0xabcddcbaull;
        input.action_dim = kDogJointCount;
        input.inference_latency_us = 123;
        FillArray(input.leg_action, 0.0f, 0.1f);

        const auto bytes = SerializeDogPolicyCommandFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kDogPolicyCommandPayloadSize, "DogPolicyCommandFrame size mismatch");

        DogPolicyCommandFrame parsed;
        std::string error;
        Require(ParseDogPolicyCommandFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::DogPolicyCommand, "DogPolicyCommandFrame type mismatch");
        Require(parsed.policy_id_hash == input.policy_id_hash, "DogPolicyCommandFrame policy hash mismatch");
        Require(parsed.action_dim == input.action_dim, "DogPolicyCommandFrame action dim mismatch");
        Require(parsed.inference_latency_us == input.inference_latency_us, "DogPolicyCommandFrame latency mismatch");
        Require(ExpectArrayEq(parsed.leg_action, input.leg_action, "leg_action"), "DogPolicyCommandFrame leg action mismatch");
    }

    {
        ArmCommandFrame input;
        input.header.msg_type = FrameType::ArmCommand;
        input.header.seq = 6;
        input.joint_count = kArmJointCount;
        input.interpolation_hint = 2;
        input.command_expire_ns = 9999;
        input.gripper_target = 0.25f;
        FillArray(input.q, 1.0f, 0.1f);
        FillArray(input.dq, 2.0f, 0.1f);
        FillArray(input.kp, 3.0f, 0.1f);
        FillArray(input.kd, 4.0f, 0.1f);
        FillArray(input.tau, 5.0f, 0.1f);

        const auto bytes = SerializeArmCommandFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kArmCommandPayloadSize, "ArmCommandFrame size mismatch");

        ArmCommandFrame parsed;
        std::string error;
        Require(ParseArmCommandFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::ArmCommand, "ArmCommandFrame type mismatch");
        Require(parsed.joint_count == input.joint_count, "ArmCommandFrame joint count mismatch");
        Require(parsed.interpolation_hint == input.interpolation_hint, "ArmCommandFrame interpolation mismatch");
        Require(parsed.command_expire_ns == input.command_expire_ns, "ArmCommandFrame expire mismatch");
        Require(parsed.gripper_target == input.gripper_target, "ArmCommandFrame gripper mismatch");
        Require(ExpectArrayEq(parsed.q, input.q, "arm_q"), "ArmCommandFrame q mismatch");
        Require(ExpectArrayEq(parsed.dq, input.dq, "arm_dq"), "ArmCommandFrame dq mismatch");
        Require(ExpectArrayEq(parsed.kp, input.kp, "arm_kp"), "ArmCommandFrame kp mismatch");
        Require(ExpectArrayEq(parsed.kd, input.kd, "arm_kd"), "ArmCommandFrame kd mismatch");
        Require(ExpectArrayEq(parsed.tau, input.tau, "arm_tau"), "ArmCommandFrame tau mismatch");
    }

    {
        BodyCommandFrame input;
        input.header.msg_type = FrameType::BodyCommand;
        input.header.seq = 7;
        input.joint_count = kBodyJointCount;
        input.command_expire_ns = 8888;
        FillArray(input.q, 1.0f, 0.2f);
        FillArray(input.dq, 2.0f, 0.2f);
        FillArray(input.kp, 3.0f, 0.2f);
        FillArray(input.kd, 4.0f, 0.2f);
        FillArray(input.tau, 5.0f, 0.2f);

        const auto bytes = SerializeBodyCommandFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kBodyCommandPayloadSize, "BodyCommandFrame size mismatch");

        BodyCommandFrame parsed;
        std::string error;
        Require(ParseBodyCommandFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::BodyCommand, "BodyCommandFrame type mismatch");
        Require(parsed.joint_count == input.joint_count, "BodyCommandFrame joint count mismatch");
        Require(parsed.command_expire_ns == input.command_expire_ns, "BodyCommandFrame expire mismatch");
        Require(ExpectArrayEq(parsed.q, input.q, "body_q"), "BodyCommandFrame q mismatch");
        Require(ExpectArrayEq(parsed.dq, input.dq, "body_dq"), "BodyCommandFrame dq mismatch");
        Require(ExpectArrayEq(parsed.kp, input.kp, "body_kp"), "BodyCommandFrame kp mismatch");
        Require(ExpectArrayEq(parsed.kd, input.kd, "body_kd"), "BodyCommandFrame kd mismatch");
        Require(ExpectArrayEq(parsed.tau, input.tau, "body_tau"), "BodyCommandFrame tau mismatch");
    }

    {
        HybridDiagnosticFrame input;
        input.header.msg_type = FrameType::HybridDiagnostic;
        input.header.seq = 8;
        input.body_state_age_us = 111;
        input.arm_state_age_us = 222;
        input.policy_latency_us = 333;
        input.coordinator_jitter_us = 444;
        input.arm_tracking_error_norm = 0.25f;
        input.xy_drift_error = 0.02f;
        input.yaw_drift_error = 0.03f;
        input.clip_count = 4;
        input.dds_write_fail_count = 5;
        input.arm_backend_fail_count = 6;
        input.seq_gap_count = 7;

        const auto bytes = SerializeHybridDiagnosticFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kHybridDiagnosticPayloadSize, "HybridDiagnosticFrame size mismatch");

        HybridDiagnosticFrame parsed;
        std::string error;
        Require(ParseHybridDiagnosticFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::HybridDiagnostic, "HybridDiagnosticFrame type mismatch");
        Require(parsed.body_state_age_us == input.body_state_age_us, "HybridDiagnosticFrame body age mismatch");
        Require(parsed.arm_state_age_us == input.arm_state_age_us, "HybridDiagnosticFrame arm age mismatch");
        Require(parsed.policy_latency_us == input.policy_latency_us, "HybridDiagnosticFrame policy latency mismatch");
        Require(parsed.coordinator_jitter_us == input.coordinator_jitter_us, "HybridDiagnosticFrame jitter mismatch");
        Require(parsed.arm_tracking_error_norm == input.arm_tracking_error_norm, "HybridDiagnosticFrame tracking error mismatch");
        Require(parsed.xy_drift_error == input.xy_drift_error, "HybridDiagnosticFrame xy drift mismatch");
        Require(parsed.yaw_drift_error == input.yaw_drift_error, "HybridDiagnosticFrame yaw drift mismatch");
        Require(parsed.clip_count == input.clip_count, "HybridDiagnosticFrame clip count mismatch");
        Require(parsed.dds_write_fail_count == input.dds_write_fail_count, "HybridDiagnosticFrame dds fail mismatch");
        Require(parsed.arm_backend_fail_count == input.arm_backend_fail_count, "HybridDiagnosticFrame arm fail mismatch");
        Require(parsed.seq_gap_count == input.seq_gap_count, "HybridDiagnosticFrame seq gap mismatch");
    }

    {
        ModeEventFrame input;
        input.header.msg_type = FrameType::ModeEvent;
        input.header.seq = 9;
        input.from_mode = 3;
        input.to_mode = 6;
        input.reason_code = 1005;
        input.trigger_seq = 77;
        input.detail_value = 99;

        const auto bytes = SerializeModeEventFrame(input);
        Require(bytes.size() == kFrameHeaderSize + kModeEventPayloadSize, "ModeEventFrame size mismatch");

        ModeEventFrame parsed;
        std::string error;
        Require(ParseModeEventFrame(bytes, parsed, &error), error.c_str());
        Require(parsed.header.msg_type == FrameType::ModeEvent, "ModeEventFrame type mismatch");
        Require(parsed.from_mode == input.from_mode, "ModeEventFrame from mode mismatch");
        Require(parsed.to_mode == input.to_mode, "ModeEventFrame to mode mismatch");
        Require(parsed.reason_code == input.reason_code, "ModeEventFrame reason mismatch");
        Require(parsed.trigger_seq == input.trigger_seq, "ModeEventFrame trigger seq mismatch");
        Require(parsed.detail_value == input.detail_value, "ModeEventFrame detail value mismatch");
    }
}

}  // namespace

int main()
{
    TestStaticShapes();
    TestHeaderRoundTrip();
    TestHelpers();
    TestFrameRoundTrips();
    std::cout << "test_go2_x5_protocol_contract passed\n";
    return 0;
}
