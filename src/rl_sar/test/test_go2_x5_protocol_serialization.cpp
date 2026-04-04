#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "rl_sar/protocol/go2_x5_protocol.hpp"

namespace
{

using namespace rl_sar::protocol;

template <typename T, size_t N>
bool ExpectArrayEq(const std::array<T, N>& lhs, const std::array<T, N>& rhs, const char* name)
{
    for (size_t i = 0; i < N; ++i)
    {
        if (lhs[i] != rhs[i])
        {
            std::cerr << name << " mismatch at index " << i << "\n";
            return false;
        }
    }
    return true;
}

template <typename T, size_t N>
void FillArray(std::array<T, N>& values, T start, T step)
{
    for (size_t i = 0; i < N; ++i)
    {
        values[i] = static_cast<T>(start + static_cast<T>(step * static_cast<T>(i)));
    }
}

bool ExpectNear(float lhs, float rhs, const char* name)
{
    if (std::fabs(lhs - rhs) > 1e-6f)
    {
        std::cerr << name << " mismatch: " << lhs << " vs " << rhs << "\n";
        return false;
    }
    return true;
}

template <typename FrameT, typename SerializeFn, typename ParseFn>
bool ExpectRoundTrip(const FrameT& input, SerializeFn serialize, ParseFn parse, const char* name)
{
    const auto bytes = serialize(input);
    FrameT output;
    std::string error;
    if (!parse(bytes, output, &error))
    {
        std::cerr << name << " parse failed: " << error << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    {
        FrameHeader header;
        header.seq = 42;
        header.source_monotonic_ns = 1000;
        header.publish_monotonic_ns = 1200;
        header.source_id = 7;
        header.mode = 3;
        header.validity_flags = kValidityPayloadValid | kValidityFromBackend;
        header.fault_flags = kFaultSequenceGap;
        header.payload_bytes = 12;
        header.msg_type = FrameType::ModeEvent;

        const auto bytes = SerializeFrameHeader(header);
        if (bytes.size() != kFrameHeaderSize)
        {
            std::cerr << "header size mismatch\n";
            return 1;
        }

        FrameHeader parsed;
        std::string error;
        if (!ParseFrameHeader(bytes, parsed, &error))
        {
            std::cerr << "header parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.magic != kFrameMagic || parsed.version != kProtocolVersion || parsed.msg_type != FrameType::ModeEvent ||
            parsed.seq != header.seq || parsed.source_monotonic_ns != header.source_monotonic_ns ||
            parsed.publish_monotonic_ns != header.publish_monotonic_ns || parsed.source_id != header.source_id ||
            parsed.mode != header.mode || parsed.validity_flags != header.validity_flags ||
            parsed.fault_flags != header.fault_flags || parsed.payload_bytes != header.payload_bytes)
        {
            std::cerr << "header roundtrip mismatch\n";
            return 1;
        }
    }

    {
        BodyStateFrame input;
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
        if (bytes.size() != kFrameHeaderSize + kBodyStatePayloadSize)
        {
            std::cerr << "BodyStateFrame size mismatch\n";
            return 1;
        }

        BodyStateFrame parsed;
        std::string error;
        if (!ParseBodyStateFrame(bytes, parsed, &error))
        {
            std::cerr << "BodyStateFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.header.msg_type != FrameType::BodyState || parsed.lowstate_age_us != input.lowstate_age_us ||
            parsed.dds_ok != input.dds_ok || parsed.reserved != input.reserved ||
            !ExpectArrayEq(parsed.imu_quat, input.imu_quat, "imu_quat") ||
            !ExpectArrayEq(parsed.imu_gyro, input.imu_gyro, "imu_gyro") ||
            !ExpectArrayEq(parsed.imu_acc, input.imu_acc, "imu_acc") ||
            !ExpectArrayEq(parsed.leg_q, input.leg_q, "leg_q") ||
            !ExpectArrayEq(parsed.leg_dq, input.leg_dq, "leg_dq") ||
            !ExpectArrayEq(parsed.leg_tau, input.leg_tau, "leg_tau") ||
            !ExpectArrayEq(parsed.base_lin_vel, input.base_lin_vel, "base_lin_vel") ||
            !ExpectArrayEq(parsed.base_ang_vel, input.base_ang_vel, "base_ang_vel") ||
            !ExpectArrayEq(parsed.projected_gravity, input.projected_gravity, "projected_gravity"))
        {
            return 1;
        }
    }

    {
        ArmStateFrame input;
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
        if (bytes.size() != kFrameHeaderSize + kArmStatePayloadSize)
        {
            std::cerr << "ArmStateFrame size mismatch\n";
            return 1;
        }

        ArmStateFrame parsed;
        std::string error;
        if (!ParseArmStateFrame(bytes, parsed, &error))
        {
            std::cerr << "ArmStateFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.header.msg_type != FrameType::ArmState || parsed.joint_count != kArmJointCount ||
            parsed.backend_mode != input.backend_mode || parsed.backend_age_us != input.backend_age_us ||
            parsed.transport_age_us != input.transport_age_us || parsed.target_seq_applied != input.target_seq_applied ||
            !ExpectArrayEq(parsed.q, input.q, "arm_q") ||
            !ExpectArrayEq(parsed.dq, input.dq, "arm_dq") ||
            !ExpectArrayEq(parsed.tau, input.tau, "arm_tau") ||
            !ExpectArrayEq(parsed.q_target, input.q_target, "arm_q_target") ||
            !ExpectArrayEq(parsed.tracking_error, input.tracking_error, "arm_tracking_error"))
        {
            return 1;
        }
    }

    {
        OperatorCommandFrame input;
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
        if (bytes.size() != kFrameHeaderSize + kOperatorCommandPayloadSize)
        {
            std::cerr << "OperatorCommandFrame size mismatch\n";
            return 1;
        }

        OperatorCommandFrame parsed;
        std::string error;
        if (!ParseOperatorCommandFrame(bytes, parsed, &error))
        {
            std::cerr << "OperatorCommandFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.cmd_source != input.cmd_source || parsed.key_mask != input.key_mask ||
            parsed.e_stop != input.e_stop || parsed.locomotion_enable != input.locomotion_enable ||
            parsed.arm_enable != input.arm_enable || !ExpectNear(parsed.cmd_vel_x, input.cmd_vel_x, "cmd_vel_x") ||
            !ExpectNear(parsed.cmd_vel_y, input.cmd_vel_y, "cmd_vel_y") ||
            !ExpectNear(parsed.cmd_vel_yaw, input.cmd_vel_yaw, "cmd_vel_yaw") ||
            !ExpectArrayEq(parsed.arm_target_q, input.arm_target_q, "arm_target_q") ||
            !ExpectNear(parsed.gripper_target, input.gripper_target, "gripper_target"))
        {
            return 1;
        }
    }

    {
        DogPolicyObservationFrame input;
        input.header.seq = 4;
        FillArray(input.base_lin_vel, 1.0f, 0.1f);
        FillArray(input.base_ang_vel, 2.0f, 0.1f);
        FillArray(input.projected_gravity, 3.0f, 0.1f);
        FillArray(input.velocity_commands, 4.0f, 0.1f);
        FillArray(input.full_joint_pos, 5.0f, 0.1f);
        FillArray(input.full_joint_vel, 6.0f, 0.1f);
        FillArray(input.last_actions, 7.0f, 0.1f);
        FillArray(input.height_scan, 8.0f, 0.1f);
        FillArray(input.arm_joint_command, 9.0f, 0.1f);
        input.gripper_command[0] = 10.0f;

        const auto bytes = SerializeDogPolicyObservationFrame(input);
        if (bytes.size() != kFrameHeaderSize + kDogPolicyObservationPayloadSize)
        {
            std::cerr << "DogPolicyObservationFrame size mismatch\n";
            return 1;
        }

        DogPolicyObservationFrame parsed;
        std::string error;
        if (!ParseDogPolicyObservationFrame(bytes, parsed, &error))
        {
            std::cerr << "DogPolicyObservationFrame parse failed: " << error << "\n";
            return 1;
        }
        if (!ExpectArrayEq(parsed.base_lin_vel, input.base_lin_vel, "obs_base_lin_vel") ||
            !ExpectArrayEq(parsed.base_ang_vel, input.base_ang_vel, "obs_base_ang_vel") ||
            !ExpectArrayEq(parsed.projected_gravity, input.projected_gravity, "obs_projected_gravity") ||
            !ExpectArrayEq(parsed.velocity_commands, input.velocity_commands, "obs_velocity_commands") ||
            !ExpectArrayEq(parsed.full_joint_pos, input.full_joint_pos, "obs_full_joint_pos") ||
            !ExpectArrayEq(parsed.full_joint_vel, input.full_joint_vel, "obs_full_joint_vel") ||
            !ExpectArrayEq(parsed.last_actions, input.last_actions, "obs_last_actions") ||
            !ExpectArrayEq(parsed.height_scan, input.height_scan, "obs_height_scan") ||
            !ExpectArrayEq(parsed.arm_joint_command, input.arm_joint_command, "obs_arm_joint_command") ||
            !ExpectArrayEq(parsed.gripper_command, input.gripper_command, "obs_gripper_command"))
        {
            return 1;
        }
    }

    {
        DogPolicyCommandFrame input;
        input.header.seq = 5;
        input.policy_id_hash = 0x1122334455667788ull;
        input.action_dim = kDogJointCount;
        input.inference_latency_us = 123;
        FillArray(input.leg_action, -0.5f, 0.05f);

        const auto bytes = SerializeDogPolicyCommandFrame(input);
        if (bytes.size() != kFrameHeaderSize + kDogPolicyCommandPayloadSize)
        {
            std::cerr << "DogPolicyCommandFrame size mismatch\n";
            return 1;
        }

        DogPolicyCommandFrame parsed;
        std::string error;
        if (!ParseDogPolicyCommandFrame(bytes, parsed, &error))
        {
            std::cerr << "DogPolicyCommandFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.policy_id_hash != input.policy_id_hash || parsed.action_dim != kDogJointCount ||
            parsed.inference_latency_us != input.inference_latency_us ||
            !ExpectArrayEq(parsed.leg_action, input.leg_action, "leg_action"))
        {
            return 1;
        }
    }

    {
        ArmCommandFrame input;
        input.header.seq = 6;
        input.joint_count = kArmJointCount;
        input.interpolation_hint = 2;
        input.command_expire_ns = 99999;
        FillArray(input.q, 0.1f, 0.1f);
        FillArray(input.dq, 0.2f, 0.1f);
        FillArray(input.kp, 1.0f, 0.2f);
        FillArray(input.kd, 2.0f, 0.2f);
        FillArray(input.tau, 3.0f, 0.2f);
        input.gripper_target = 0.75f;

        const auto bytes = SerializeArmCommandFrame(input);
        if (bytes.size() != kFrameHeaderSize + kArmCommandPayloadSize)
        {
            std::cerr << "ArmCommandFrame size mismatch\n";
            return 1;
        }

        ArmCommandFrame parsed;
        std::string error;
        if (!ParseArmCommandFrame(bytes, parsed, &error))
        {
            std::cerr << "ArmCommandFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.joint_count != kArmJointCount || parsed.interpolation_hint != input.interpolation_hint ||
            parsed.command_expire_ns != input.command_expire_ns || !ExpectNear(parsed.gripper_target, input.gripper_target, "arm_gripper") ||
            !ExpectArrayEq(parsed.q, input.q, "arm_cmd_q") ||
            !ExpectArrayEq(parsed.dq, input.dq, "arm_cmd_dq") ||
            !ExpectArrayEq(parsed.kp, input.kp, "arm_cmd_kp") ||
            !ExpectArrayEq(parsed.kd, input.kd, "arm_cmd_kd") ||
            !ExpectArrayEq(parsed.tau, input.tau, "arm_cmd_tau"))
        {
            return 1;
        }
    }

    {
        BodyCommandFrame input;
        input.header.seq = 7;
        input.command_expire_ns = 88888;
        FillArray(input.q, 0.0f, 0.25f);
        FillArray(input.dq, 1.0f, 0.25f);
        FillArray(input.kp, 2.0f, 0.25f);
        FillArray(input.kd, 3.0f, 0.25f);
        FillArray(input.tau, 4.0f, 0.25f);

        const auto bytes = SerializeBodyCommandFrame(input);
        if (bytes.size() != kFrameHeaderSize + kBodyCommandPayloadSize)
        {
            std::cerr << "BodyCommandFrame size mismatch\n";
            return 1;
        }

        BodyCommandFrame parsed;
        std::string error;
        if (!ParseBodyCommandFrame(bytes, parsed, &error))
        {
            std::cerr << "BodyCommandFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.joint_count != kBodyJointCount || parsed.command_expire_ns != input.command_expire_ns ||
            !ExpectArrayEq(parsed.q, input.q, "body_cmd_q") ||
            !ExpectArrayEq(parsed.dq, input.dq, "body_cmd_dq") ||
            !ExpectArrayEq(parsed.kp, input.kp, "body_cmd_kp") ||
            !ExpectArrayEq(parsed.kd, input.kd, "body_cmd_kd") ||
            !ExpectArrayEq(parsed.tau, input.tau, "body_cmd_tau"))
        {
            return 1;
        }
    }

    {
        HybridDiagnosticFrame input;
        input.header.seq = 8;
        input.body_state_age_us = 10;
        input.arm_state_age_us = 20;
        input.policy_latency_us = 30;
        input.coordinator_jitter_us = 40;
        input.arm_tracking_error_norm = 1.5f;
        input.xy_drift_error = 2.5f;
        input.yaw_drift_error = 3.5f;
        input.clip_count = 4;
        input.dds_write_fail_count = 5;
        input.arm_backend_fail_count = 6;
        input.seq_gap_count = 7;

        const auto bytes = SerializeHybridDiagnosticFrame(input);
        if (bytes.size() != kFrameHeaderSize + kHybridDiagnosticPayloadSize)
        {
            std::cerr << "HybridDiagnosticFrame size mismatch\n";
            return 1;
        }

        HybridDiagnosticFrame parsed;
        std::string error;
        if (!ParseHybridDiagnosticFrame(bytes, parsed, &error))
        {
            std::cerr << "HybridDiagnosticFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.body_state_age_us != input.body_state_age_us || parsed.arm_state_age_us != input.arm_state_age_us ||
            parsed.policy_latency_us != input.policy_latency_us || parsed.coordinator_jitter_us != input.coordinator_jitter_us ||
            !ExpectNear(parsed.arm_tracking_error_norm, input.arm_tracking_error_norm, "arm_tracking_error_norm") ||
            !ExpectNear(parsed.xy_drift_error, input.xy_drift_error, "xy_drift_error") ||
            !ExpectNear(parsed.yaw_drift_error, input.yaw_drift_error, "yaw_drift_error") ||
            parsed.clip_count != input.clip_count || parsed.dds_write_fail_count != input.dds_write_fail_count ||
            parsed.arm_backend_fail_count != input.arm_backend_fail_count || parsed.seq_gap_count != input.seq_gap_count)
        {
            return 1;
        }
    }

    {
        ModeEventFrame input;
        input.header.seq = 9;
        input.from_mode = 3;
        input.to_mode = 4;
        input.reason_code = 1004;
        input.trigger_seq = 99;
        input.detail_value = 77;

        const auto bytes = SerializeModeEventFrame(input);
        if (bytes.size() != kFrameHeaderSize + kModeEventPayloadSize)
        {
            std::cerr << "ModeEventFrame size mismatch\n";
            return 1;
        }

        ModeEventFrame parsed;
        std::string error;
        if (!ParseModeEventFrame(bytes, parsed, &error))
        {
            std::cerr << "ModeEventFrame parse failed: " << error << "\n";
            return 1;
        }
        if (parsed.from_mode != input.from_mode || parsed.to_mode != input.to_mode ||
            parsed.reason_code != input.reason_code || parsed.trigger_seq != input.trigger_seq ||
            parsed.detail_value != input.detail_value)
        {
            std::cerr << "ModeEventFrame roundtrip mismatch\n";
            return 1;
        }
    }

    {
        const uint64_t source_ns = 1000;
        if (AgeNs(1500, source_ns) != 500 || !IsFresh(1500, source_ns, 500) || IsFresh(1500, source_ns, 499))
        {
            std::cerr << "Freshness helper mismatch\n";
            return 1;
        }
        if (SeqGap(10, 12) != 2 || HasSeqGap(10, 11) || !HasSeqGap(10, 12))
        {
            std::cerr << "Seq helper mismatch\n";
            return 1;
        }
        if (!IsCommandExpired(1000, 999) || IsCommandExpired(1000, 1001))
        {
            std::cerr << "Expiry helper mismatch\n";
            return 1;
        }
    }

    {
        BodyCommandFrame parsed;
        std::string error;
        std::vector<uint8_t> invalid = {0, 1, 2, 3, 4, 5, 6, 7};
        if (ParseBodyCommandFrame(invalid, parsed, &error))
        {
            std::cerr << "Expected short packet rejection\n";
            return 1;
        }
    }

    {
        BodyCommandFrame input;
        const auto bytes = SerializeBodyCommandFrame(input);
        std::vector<uint8_t> invalid = bytes;
        invalid[0] = 0x00;
        BodyCommandFrame parsed;
        std::string error;
        if (ParseBodyCommandFrame(invalid, parsed, &error))
        {
            std::cerr << "Expected bad magic rejection\n";
            return 1;
        }
    }

    {
        BodyCommandFrame input;
        const auto bytes = SerializeBodyCommandFrame(input);
        std::vector<uint8_t> invalid = bytes;
        invalid[4] = 0x02;  // version low byte
        BodyCommandFrame parsed;
        std::string error;
        if (ParseBodyCommandFrame(invalid, parsed, &error))
        {
            std::cerr << "Expected bad version rejection\n";
            return 1;
        }
    }

    std::cout << "test_go2_x5_protocol_serialization passed\n";
    return 0;
}
