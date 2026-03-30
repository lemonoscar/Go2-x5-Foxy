#include <cmath>
#include <iostream>
#include <vector>

#include "rl_sar/go2x5/arm/go2_x5_arm_runtime.hpp"

namespace
{

bool Near(float a, float b, float eps = 1e-5f)
{
    return std::fabs(a - b) <= eps;
}

bool RequireNearVector(const std::vector<float>& actual,
                       const std::vector<float>& expected,
                       const char* label)
{
    if (actual.size() != expected.size())
    {
        std::cerr << label << " size mismatch\n";
        return false;
    }
    for (size_t i = 0; i < actual.size(); ++i)
    {
        if (!Near(actual[i], expected[i]))
        {
            std::cerr << label << " mismatch at index " << i << ": actual=" << actual[i]
                      << ", expected=" << expected[i] << "\n";
            return false;
        }
    }
    return true;
}

} // namespace

int main()
{
    using namespace Go2X5ArmRuntime;

    {
        InitializationConfig config;
        config.arm_command_size = 3;
        config.arm_joint_start_index = 2;
        config.arm_hold_enabled = true;
        config.step_dt = 0.02f;
        config.smoothing_time = 0.10f;
        config.default_dof_pos = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

        const auto state = BuildInitialCommandState(config);
        if (state.arm_command_smoothing_ticks != 5)
        {
            std::cerr << "Expected smoothing ticks to be derived from step_dt and smoothing_time\n";
            return 1;
        }
        if (!RequireNearVector(state.arm_hold_position, {2.0f, 3.0f, 4.0f}, "init hold"))
        {
            return 1;
        }
        if (!RequireNearVector(state.arm_joint_command_latest, state.arm_hold_position, "init latest"))
        {
            return 1;
        }
    }

    {
        InitializationConfig config;
        config.arm_command_size = 2;
        config.arm_joint_start_index = 5;
        config.default_dof_pos = {10.0f, 11.0f, 12.0f};
        const auto state = BuildInitialCommandState(config);
        if (!RequireNearVector(state.arm_hold_position, {11.0f, 12.0f}, "fallback tail hold"))
        {
            return 1;
        }
    }

    {
        InitializationConfig config;
        config.arm_command_size = 2;
        config.arm_hold_pose = {0.0f, 0.0f};
        config.step_dt = 0.02f;
        config.smoothing_time = 0.10f;
        auto state = BuildInitialCommandState(config);

        auto obs = StepSmoothedCommand(&state, {1.0f, 2.0f});
        if (!RequireNearVector(obs, {0.2f, 0.4f}, "smooth step 1"))
        {
            return 1;
        }
        obs = StepSmoothedCommand(&state, {1.0f, 2.0f});
        if (!RequireNearVector(obs, {0.4f, 0.8f}, "smooth step 2"))
        {
            return 1;
        }

        ApplyHoldTarget(&state, {0.5f, -0.5f});
        if (!RequireNearVector(state.arm_hold_position, {0.5f, -0.5f}, "apply hold"))
        {
            return 1;
        }
        if (!RequireNearVector(state.arm_joint_command_latest, {0.5f, -0.5f}, "apply hold latest"))
        {
            return 1;
        }
        if (state.arm_command_smoothing_counter != 0)
        {
            std::cerr << "Expected hold update to restart smoothing\n";
            return 1;
        }
    }

    {
        InitializationConfig config;
        config.arm_command_size = 2;
        config.arm_hold_pose = {1.0f, 1.0f};
        auto state = BuildInitialCommandState(config);
        auto previous = CaptureSnapshot(state);
        previous.hold_position = {0.2f, 0.3f};
        previous.joint_command_latest = {0.2f, 0.3f};
        previous.topic_command_latest = {0.4f, 0.5f};
        previous.topic_command_received = true;
        previous.command_initialized = true;
        previous.command_smoothing_counter = 3;
        previous.command_smoothing_start = {1.0f, 1.0f};
        previous.command_smoothing_target = {0.2f, 0.3f};
        previous.command_smoothed = {0.6f, 0.65f};

        RestoreSnapshotIfCompatible(&state, previous);
        if (!RequireNearVector(state.arm_hold_position, {0.2f, 0.3f}, "restore hold"))
        {
            return 1;
        }
        if (!state.arm_topic_command_received || state.arm_command_smoothing_counter != 3)
        {
            std::cerr << "Expected compatible snapshot restore to preserve runtime flags\n";
            return 1;
        }
    }

    std::cout << "go2_x5 arm runtime test passed." << std::endl;
    return 0;
}
