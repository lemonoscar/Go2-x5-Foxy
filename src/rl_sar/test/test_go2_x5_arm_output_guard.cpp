#include <iostream>
#include <vector>

#include "rl_sar/go2x5/arm/go2_x5_arm_output_guard.hpp"

namespace
{

bool RequireEqual(const std::vector<float>& actual,
                  const std::vector<float>& expected,
                  const char* label)
{
    if (actual != expected)
    {
        std::cerr << label << " mismatch\n";
        return false;
    }
    return true;
}

} // namespace

int main()
{
    using namespace Go2X5ArmOutputGuard;

    {
        Context context;
        context.num_dofs = 8;
        context.arm_joint_start_index = 2;
        context.arm_command_size = 3;
        context.arm_hold_enabled = true;
        context.arm_hold_position = {10.0f, 11.0f, 12.0f};

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
        std::vector<float> output_vel = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f};
        if (!ApplyOutputGuards(&output_pos, &output_vel, context))
        {
            std::cerr << "Expected hold override to apply\n";
            return 1;
        }
        if (!RequireEqual(output_pos, {0.0f, 1.0f, 10.0f, 11.0f, 12.0f, 5.0f, 6.0f, 7.0f}, "hold pos"))
        {
            return 1;
        }
        if (!RequireEqual(output_vel, {0.1f, 0.2f, 0.0f, 0.0f, 0.0f, 0.6f, 0.7f, 0.8f}, "hold vel"))
        {
            return 1;
        }
    }

    {
        Context context;
        context.num_dofs = 8;
        context.arm_joint_start_index = 1;
        context.arm_command_size = 3;
        context.arm_hold_enabled = false;
        context.arm_split_control_enabled = true;
        context.arm_bridge_require_state = true;
        context.arm_bridge_state_fresh = false;
        context.arm_hold_position = {4.0f, 5.0f, 6.0f};

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
        std::vector<float> output_vel = {1.0f, 1.0f};
        if (!ApplyOutputGuards(&output_pos, &output_vel, context))
        {
            std::cerr << "Expected stale bridge override to apply\n";
            return 1;
        }
        if (!RequireEqual(output_pos, {0.0f, 4.0f, 5.0f, 6.0f, 4.0f, 5.0f, 6.0f, 7.0f}, "stale bridge pos"))
        {
            return 1;
        }
        if (!RequireEqual(output_vel, {1.0f, 0.0f}, "stale bridge vel"))
        {
            return 1;
        }
    }

    {
        Context context;
        context.num_dofs = 8;
        context.arm_joint_start_index = 2;
        context.arm_command_size = 3;
        context.arm_split_control_enabled = true;
        context.arm_bridge_require_state = true;
        context.arm_bridge_state_fresh = true;
        context.arm_hold_position = {7.0f, 8.0f, 9.0f};

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
        std::vector<float> output_vel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        if (ApplyOutputGuards(&output_pos, &output_vel, context))
        {
            std::cerr << "Did not expect override when bridge state is fresh and hold is disabled\n";
            return 1;
        }
        if (!RequireEqual(output_pos, {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f}, "fresh bridge pos"))
        {
            return 1;
        }
    }

    {
        Context context;
        context.num_dofs = 4;
        context.arm_joint_start_index = 3;
        context.arm_command_size = 2;
        context.arm_hold_enabled = true;
        context.arm_hold_position = {1.0f, 2.0f};

        std::vector<float> output_pos = {0.0f, 1.0f, 2.0f, 3.0f};
        std::vector<float> output_vel = {0.0f, 0.0f, 0.0f, 0.0f};
        if (ApplyOutputGuards(&output_pos, &output_vel, context))
        {
            std::cerr << "Did not expect override for invalid arm segment range\n";
            return 1;
        }
    }

    std::cout << "go2_x5 arm output guard test passed." << std::endl;
    return 0;
}
