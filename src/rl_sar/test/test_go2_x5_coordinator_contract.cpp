#include <cstdlib>
#include <iostream>

#include "rl_sar/runtime/coordinator/go2_x5_coordinator.hpp"

namespace
{

using Go2X5Supervisor::Mode;
using rl_sar::protocol::ArmCommandFrame;
using rl_sar::protocol::ArmStateFrame;
using rl_sar::protocol::BodyStateFrame;
using rl_sar::protocol::DogPolicyCommandFrame;
using rl_sar::runtime::coordinator::Config;
using rl_sar::runtime::coordinator::HybridMotionCoordinator;
using rl_sar::runtime::coordinator::Input;

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

Config MakeConfig()
{
    Config config;
    config.body_command_expire_ns = 20'000'000ULL;
    config.arm_command_expire_ns = 15'000'000ULL;
    for (size_t i = 0; i < rl_sar::protocol::kBodyJointCount; ++i)
    {
        config.default_leg_q[i] = 0.1f * static_cast<float>(i + 1);
        config.safe_stand_q[i] = config.default_leg_q[i];
        config.action_scale[i] = 0.5f;
        config.rl_kp[i] = 20.0f;
        config.rl_kd[i] = 1.0f;
        config.torque_limits[i] = 100.0f;
    }
    return config;
}

BodyStateFrame MakeBodyState()
{
    BodyStateFrame frame;
    frame.header.seq = 10;
    frame.header.source_monotonic_ns = 1'000'000ULL;
    for (size_t i = 0; i < rl_sar::protocol::kBodyJointCount; ++i)
    {
        frame.leg_q[i] = 0.05f * static_cast<float>(i);
        frame.leg_dq[i] = 0.01f * static_cast<float>(i);
    }
    return frame;
}

DogPolicyCommandFrame MakePolicyCommand(uint64_t now_ns)
{
    DogPolicyCommandFrame frame;
    frame.header.seq = 20;
    frame.header.source_monotonic_ns = now_ns;
    frame.action_dim = rl_sar::protocol::kDogJointCount;
    for (size_t i = 0; i < rl_sar::protocol::kDogJointCount; ++i)
    {
        frame.leg_action[i] = 0.2f;
    }
    return frame;
}

ArmCommandFrame MakeArmCommand(uint64_t now_ns)
{
    ArmCommandFrame frame;
    frame.header.seq = 30;
    frame.header.source_monotonic_ns = now_ns;
    frame.joint_count = rl_sar::protocol::kArmJointCount;
    frame.command_expire_ns = now_ns + 15'000'000ULL;
    for (size_t i = 0; i < rl_sar::protocol::kArmJointCount; ++i)
    {
        frame.q[i] = 0.3f * static_cast<float>(i + 1);
        frame.kp[i] = 10.0f;
        frame.kd[i] = 1.0f;
    }
    return frame;
}

ArmStateFrame MakeArmState()
{
    ArmStateFrame frame;
    frame.header.seq = 40;
    frame.header.source_monotonic_ns = 1'000'000ULL;
    for (size_t i = 0; i < rl_sar::protocol::kArmJointCount; ++i)
    {
        frame.q[i] = 0.4f * static_cast<float>(i + 1);
    }
    return frame;
}

void TestReadyOnlyAllowsArm()
{
    HybridMotionCoordinator coordinator(MakeConfig());
    Input input;
    input.now_monotonic_ns = 5'000'000ULL;
    input.mode = Mode::Ready;
    input.has_arm_command = true;
    input.arm_command = MakeArmCommand(input.now_monotonic_ns);

    const auto output = coordinator.Step(input);
    Require(!output.body_command_valid, "READY should not emit body locomotion command");
    Require(output.arm_command_valid, "READY should allow arm passthrough");
    Require(output.arm_passthrough_applied, "READY should mark arm passthrough applied");
}

void TestRlActiveDecodesDogOnlyBody()
{
    HybridMotionCoordinator coordinator(MakeConfig());
    Input input;
    input.now_monotonic_ns = 6'000'000ULL;
    input.mode = Mode::RlDogOnlyActive;
    input.has_body_state = true;
    input.body_state = MakeBodyState();
    input.has_policy_command = true;
    input.dog_policy_command = MakePolicyCommand(input.now_monotonic_ns);
    input.has_arm_command = true;
    input.arm_command = MakeArmCommand(input.now_monotonic_ns);

    const auto output = coordinator.Step(input);
    Require(output.body_command_valid, "RL active should emit body command");
    Require(output.policy_applied, "RL active should mark policy applied");
    Require(output.arm_command_valid, "RL active should allow arm passthrough when healthy");
    Require(output.body_command.q[0] > 0.0f, "body target q should be decoded from policy");
    Require(output.body_command.kp[0] == 20.0f, "body kp should come from coordinator config");
}

void TestExpiredArmCommandRejected()
{
    HybridMotionCoordinator coordinator(MakeConfig());
    Input input;
    input.now_monotonic_ns = 7'000'000ULL;
    input.mode = Mode::RlDogOnlyActive;
    input.has_arm_command = true;
    input.arm_command = MakeArmCommand(1'000'000ULL);
    input.arm_command.command_expire_ns = 2'000'000ULL;

    const auto output = coordinator.Step(input);
    Require(!output.arm_command_valid, "expired arm command should be rejected");
}

void TestDegradedArmForcesHold()
{
    HybridMotionCoordinator coordinator(MakeConfig());
    Input input;
    input.now_monotonic_ns = 8'000'000ULL;
    input.mode = Mode::DegradedArm;
    input.has_body_state = true;
    input.body_state = MakeBodyState();
    input.has_arm_state = true;
    input.arm_state = MakeArmState();

    const auto output = coordinator.Step(input);
    Require(output.body_command_valid, "DEGRADED_ARM should emit safe stand body command");
    Require(output.body_hold, "DEGRADED_ARM should mark body hold");
    Require(output.arm_command_valid, "DEGRADED_ARM should emit arm hold command");
    Require(output.arm_hold, "DEGRADED_ARM should mark arm hold");
    Require(output.arm_command.q[0] == input.arm_state.q[0], "arm hold should latch current arm pose");
}

}  // namespace

int main()
{
    TestReadyOnlyAllowsArm();
    TestRlActiveDecodesDogOnlyBody();
    TestExpiredArmCommandRejected();
    TestDegradedArmForcesHold();
    std::cout << "test_go2_x5_coordinator_contract passed\n";
    return 0;
}
