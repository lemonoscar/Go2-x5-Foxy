#include <cstdlib>
#include <iostream>
#include <vector>

#include "rl_sar/go2x5/arm/go2_x5_arm_output_guard.hpp"

namespace
{

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

void RequireEqual(const std::vector<float>& actual,
                  const std::vector<float>& expected,
                  const char* message)
{
    if (actual != expected)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

Go2X5ArmOutputGuard::Context MakeContext()
{
    Go2X5ArmOutputGuard::Context context;
    context.num_dofs = 8;
    context.arm_joint_start_index = 2;
    context.arm_command_size = 3;
    context.arm_hold_position = {10.0f, 11.0f, 12.0f};
    return context;
}

} // namespace

int main()
{
    {
        auto context = MakeContext();
        context.arm_hold_enabled = true;

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
        std::vector<float> output_vel = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f};
        Require(Go2X5ArmOutputGuard::ApplyOutputGuards(&output_pos, &output_vel, context),
                "enabled arm hold should apply output guard");
        RequireEqual(output_pos, {0.0f, 1.0f, 10.0f, 11.0f, 12.0f, 5.0f, 6.0f, 7.0f},
                     "enabled arm hold should override only arm joint positions");
        RequireEqual(output_vel, {0.1f, 0.2f, 0.0f, 0.0f, 0.0f, 0.6f, 0.7f, 0.8f},
                     "enabled arm hold should zero only arm joint velocities");
    }

    {
        auto context = MakeContext();
        context.arm_hold_enabled = false;
        context.arm_split_control_enabled = true;
        context.arm_bridge_require_state = true;
        context.arm_bridge_state_fresh = false;

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
        std::vector<float> output_vel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        Require(Go2X5ArmOutputGuard::ApplyOutputGuards(&output_pos, &output_vel, context),
                "stale required arm bridge state should force hold override");
        RequireEqual(output_pos, {0.0f, 1.0f, 10.0f, 11.0f, 12.0f, 5.0f, 6.0f, 7.0f},
                     "stale bridge override should hold arm pose");
    }

    {
        auto context = MakeContext();
        context.arm_hold_enabled = false;
        context.arm_split_control_enabled = true;
        context.arm_bridge_require_state = true;
        context.arm_bridge_state_fresh = true;

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
        std::vector<float> output_vel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        Require(!Go2X5ArmOutputGuard::ApplyOutputGuards(&output_pos, &output_vel, context),
                "fresh arm bridge state with hold disabled should not override output");
        RequireEqual(output_pos, {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f},
                     "fresh bridge state should leave arm output unchanged");
    }

    {
        auto context = MakeContext();
        context.num_dofs = 4;
        context.arm_joint_start_index = 3;
        context.arm_command_size = 2;
        context.arm_hold_enabled = true;
        context.arm_hold_position = {1.0f, 2.0f};

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f};
        std::vector<float> output_vel = {0.0f, 0.0f, 0.0f, 0.0f};
        Require(!Go2X5ArmOutputGuard::ApplyOutputGuards(&output_pos, &output_vel, context),
                "invalid arm joint range should not apply output guard");
    }

    std::cout << "go2_x5 arm safety guards behavior test passed." << std::endl;
    return 0;
}
