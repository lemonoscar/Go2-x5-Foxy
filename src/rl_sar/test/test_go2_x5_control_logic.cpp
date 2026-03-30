#include <iostream>
#include <vector>

#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"

int main()
{
    using namespace Go2X5ControlLogic;

    {
        const auto mode = ResolveKey1Mode(true);
        if (mode != Key1Mode::Navigation)
        {
            std::cerr << "Expected Key1Mode::Navigation when prefer_navigation_mode=true\n";
            return 1;
        }
    }

    {
        const auto mode = ResolveKey1Mode(false);
        if (mode != Key1Mode::FixedCommand)
        {
            std::cerr << "Expected Key1Mode::FixedCommand when prefer_navigation_mode=false\n";
            return 1;
        }
    }

    {
        Key1NavigationPublishRequest request;
        request.navigation_mode = true;
        request.key1_pressed = true;
        if (!ShouldPublishKey1NavigationCmd(request))
        {
            std::cerr << "Expected Key1 navigation command to publish when navigation is active and key1 is pressed\n";
            return 1;
        }
        const auto cmd = BuildKey1NavigationCommand(0.6f, -0.1f, 0.2f);
        if (cmd.x != 0.6f || cmd.y != -0.1f || cmd.yaw != 0.2f)
        {
            std::cerr << "Expected Key1 navigation command builder to preserve configured values\n";
            return 1;
        }
    }

    {
        Key1NavigationPublishRequest request;
        request.navigation_mode = true;
        request.key1_pressed = true;
        request.exclusive_control = true;
        if (ShouldPublishKey1NavigationCmd(request))
        {
            std::cerr << "Expected exclusive control to suppress Key1 navigation publish\n";
            return 1;
        }
        request.exclusive_control = false;
        request.already_published = true;
        if (ShouldPublishKey1NavigationCmd(request))
        {
            std::cerr << "Expected duplicate Key1 navigation publish to be suppressed\n";
            return 1;
        }
        request.already_published = false;
        request.publish_enabled = false;
        if (ShouldPublishKey1NavigationCmd(request))
        {
            std::cerr << "Expected disabled Key1 navigation publish flag to suppress output\n";
            return 1;
        }
        if (!ShouldResetKey1NavigationPublished(false) || ShouldResetKey1NavigationPublished(true))
        {
            std::cerr << "Expected publish latch reset only when navigation mode is off\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        const std::vector<float> key = {1, 1, 1, 1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, true, true, topic, key, hold);
        if (selected.source != ArmPoseSource::TopicCommand || selected.pose != topic)
        {
            std::cerr << "Expected key2 to prefer topic arm command\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {0.1f, 0.2f, 0.3f};
        const std::vector<float> key = {1, 1, 1, 1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, true, false, topic, key, hold);
        if (selected.source != ArmPoseSource::KeyPose || selected.pose != key)
        {
            std::cerr << "Expected key pose fallback when topic is unavailable\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {};
        const std::vector<float> key = {1, 1, 1};
        const std::vector<float> hold = {2, 2, 2, 2, 2, 2};
        const auto selected = SelectKey2ArmPose(6, true, false, topic, key, hold);
        if (selected.source != ArmPoseSource::HoldPose || selected.pose != hold)
        {
            std::cerr << "Expected hold pose fallback when key pose is too short\n";
            return 1;
        }
    }

    {
        const std::vector<float> topic = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        const std::vector<float> key = {};
        const std::vector<float> hold = {};
        const auto selected = SelectKey2ArmPose(6, false, true, topic, key, hold);
        if (selected.source != ArmPoseSource::None || !selected.pose.empty())
        {
            std::cerr << "Expected no key2 target when topic is disabled and no fallback poses exist\n";
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
