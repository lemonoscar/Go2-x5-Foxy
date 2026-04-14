#include <iostream>
#include <vector>

#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"

int main()
{
    using namespace Go2X5ControlLogic;

    {
        const std::vector<float> key = {1, 1, 1, 1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, key, hold);
        if (selected.source != ArmPoseSource::KeyPose || selected.pose != key)
        {
            std::cerr << "Expected Key[2] to prefer arm_key_pose when available\n";
            return 1;
        }
    }

    {
        const std::vector<float> key = {1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, key, hold);
        if (selected.source != ArmPoseSource::HoldPose || selected.pose != hold)
        {
            std::cerr << "Expected Key[2] to fall back to arm_hold_pose when arm_key_pose is too short\n";
            return 1;
        }
    }

    {
        const std::vector<float> key;
        const std::vector<float> hold;
        const auto selected = SelectKey2ArmPose(6, key, hold);
        if (selected.source != ArmPoseSource::None || !selected.pose.empty())
        {
            std::cerr << "Expected Key[2] to produce no arm target when both key/home poses are unavailable\n";
            return 1;
        }
    }

    {
        ArmRuntimeStateSnapshot current;
        current.arm_size = 6;
        current.hold_enabled = true;
        current.hold_position = {0, 1.57f, 1.57f, 0, 0, 0};
        current.joint_command_latest = current.hold_position;
        current.topic_command_latest = current.hold_position;

        ArmRuntimeStateSnapshot previous;
        previous.arm_size = 6;
        previous.hold_enabled = true;
        previous.topic_command_received = true;
        previous.command_initialized = true;
        previous.command_smoothing_counter = 7;
        previous.hold_position = {0.6f, 2.9f, 1.1f, 0.2f, 0.0f, 0.0f};
        previous.joint_command_latest = previous.hold_position;
        previous.topic_command_latest = {0.2f, 1.8f, 1.4f, 0.0f, 0.1f, 0.1f};
        previous.command_smoothing_start = {0, 1.57f, 1.57f, 0, 0, 0};
        previous.command_smoothing_target = previous.hold_position;
        previous.command_smoothed = {0.3f, 2.2f, 1.3f, 0.1f, 0.0f, 0.0f};

        RestoreArmRuntimeStateIfCompatible(current, previous);

        if (current.hold_position != previous.hold_position ||
            current.joint_command_latest != previous.joint_command_latest ||
            current.topic_command_latest != previous.topic_command_latest ||
            !current.topic_command_received ||
            !current.command_initialized ||
            current.command_smoothing_counter != previous.command_smoothing_counter ||
            current.command_smoothed != previous.command_smoothed)
        {
            std::cerr << "Expected arm runtime state to be preserved across compatible reinit\n";
            return 1;
        }
    }

    {
        ArmRuntimeStateSnapshot current;
        current.arm_size = 6;
        current.hold_enabled = true;
        current.hold_position = {0, 1.57f, 1.57f, 0, 0, 0};
        current.joint_command_latest = current.hold_position;
        current.topic_command_latest = current.hold_position;

        ArmRuntimeStateSnapshot previous;
        previous.arm_size = 5;
        previous.hold_enabled = false;
        previous.topic_command_received = true;
        previous.hold_position = {9, 9, 9, 9, 9};
        previous.joint_command_latest = previous.hold_position;
        previous.topic_command_latest = previous.hold_position;

        RestoreArmRuntimeStateIfCompatible(current, previous);

        if (!current.hold_enabled ||
            current.hold_position != std::vector<float>({0, 1.57f, 1.57f, 0, 0, 0}) ||
            current.topic_command_received)
        {
            std::cerr << "Expected incompatible arm runtime state to be ignored\n";
            return 1;
        }
    }

    std::cout << "test_go2_x5_control_logic passed\n";
    return 0;
}
