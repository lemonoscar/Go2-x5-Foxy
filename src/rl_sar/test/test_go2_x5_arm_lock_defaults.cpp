#include <cstdlib>
#include <iostream>
#include <vector>

#include "rl_sar/go2x5/arm/go2_x5_arm_runtime.hpp"

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

} // namespace

int main()
{
    Go2X5ArmRuntime::InitializationConfig config;
    config.arm_command_size = 3;
    config.arm_joint_start_index = 2;
    config.arm_hold_enabled = false;
    config.step_dt = 0.01f;
    config.smoothing_time = 0.03f;
    config.default_dof_pos = {0.0f, 1.0f, 10.0f, 11.0f, 12.0f, 5.0f};

    auto state = Go2X5ArmRuntime::BuildInitialCommandState(config);
    Require(state.arm_command_size == 3, "arm command size should come from initialization config");
    Require(!state.arm_hold_enabled, "initial arm hold enabled flag should preserve config default");
    Require(state.arm_command_initialized, "initial arm command should be initialized");
    Require(!state.arm_topic_command_received, "initial arm topic command should not be marked received");
    RequireEqual(state.arm_hold_position, {10.0f, 11.0f, 12.0f},
                 "hold position should resolve from default dof arm segment");
    RequireEqual(state.arm_joint_command_latest, state.arm_hold_position,
                 "latest arm command should start at hold position");

    Go2X5ArmRuntime::ApplyHoldTarget(&state, {4.0f, 5.0f, 6.0f});
    Require(state.arm_hold_enabled, "ApplyHoldTarget should enable arm hold");
    RequireEqual(state.arm_hold_position, {4.0f, 5.0f, 6.0f},
                 "ApplyHoldTarget should update hold pose");
    RequireEqual(state.arm_joint_command_latest, {4.0f, 5.0f, 6.0f},
                 "ApplyHoldTarget should update latest arm command");
    RequireEqual(state.arm_command_smoothing_target, {4.0f, 5.0f, 6.0f},
                 "ApplyHoldTarget should update smoothing target");

    std::cout << "go2_x5 arm lock defaults behavior test passed." << std::endl;
    return 0;
}
