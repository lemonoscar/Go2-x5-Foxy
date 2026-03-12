#ifndef GO2_X5_CONTROL_LOGIC_HPP
#define GO2_X5_CONTROL_LOGIC_HPP

#include <vector>

namespace Go2X5ControlLogic
{

enum class Key1Mode
{
    FixedCommand,
    Navigation
};

enum class ArmPoseSource
{
    None,
    TopicCommand,
    KeyPose,
    HoldPose
};

struct ArmPoseSelection
{
    std::vector<float> pose;
    ArmPoseSource source = ArmPoseSource::None;
};

struct ArmRuntimeStateSnapshot
{
    int arm_size = 0;
    int command_smoothing_counter = 0;
    bool hold_enabled = true;
    bool topic_command_received = false;
    bool command_initialized = false;
    std::vector<float> hold_position;
    std::vector<float> joint_command_latest;
    std::vector<float> topic_command_latest;
    std::vector<float> command_smoothing_start;
    std::vector<float> command_smoothing_target;
    std::vector<float> command_smoothed;
};

inline Key1Mode ResolveKey1Mode(bool prefer_navigation_mode)
{
    return prefer_navigation_mode ? Key1Mode::Navigation : Key1Mode::FixedCommand;
}

inline bool HasEnoughPose(const std::vector<float>& pose, int arm_size)
{
    return arm_size > 0 && pose.size() >= static_cast<size_t>(arm_size);
}

inline bool HasExactPose(const std::vector<float>& pose, int arm_size)
{
    return arm_size > 0 && pose.size() == static_cast<size_t>(arm_size);
}

inline std::vector<float> TrimPose(const std::vector<float>& pose, int arm_size)
{
    if (!HasEnoughPose(pose, arm_size))
    {
        return {};
    }
    return std::vector<float>(pose.begin(), pose.begin() + arm_size);
}

inline ArmPoseSelection SelectKey2ArmPose(
    int arm_size,
    bool prefer_topic_command,
    bool topic_command_received,
    const std::vector<float>& topic_command_pose,
    const std::vector<float>& key_pose,
    const std::vector<float>& hold_pose)
{
    ArmPoseSelection selection;
    if (arm_size <= 0)
    {
        return selection;
    }

    if (prefer_topic_command && topic_command_received && HasEnoughPose(topic_command_pose, arm_size))
    {
        selection.pose = TrimPose(topic_command_pose, arm_size);
        selection.source = ArmPoseSource::TopicCommand;
        return selection;
    }

    if (HasEnoughPose(key_pose, arm_size))
    {
        selection.pose = TrimPose(key_pose, arm_size);
        selection.source = ArmPoseSource::KeyPose;
        return selection;
    }

    if (HasEnoughPose(hold_pose, arm_size))
    {
        selection.pose = TrimPose(hold_pose, arm_size);
        selection.source = ArmPoseSource::HoldPose;
        return selection;
    }

    return selection;
}

inline void RestoreArmRuntimeStateIfCompatible(
    ArmRuntimeStateSnapshot& current,
    const ArmRuntimeStateSnapshot& previous)
{
    if (current.arm_size <= 0 || previous.arm_size != current.arm_size)
    {
        return;
    }

    current.hold_enabled = previous.hold_enabled;

    if (HasExactPose(previous.hold_position, current.arm_size))
    {
        current.hold_position = previous.hold_position;
    }

    if (HasExactPose(previous.joint_command_latest, current.arm_size))
    {
        current.joint_command_latest = previous.joint_command_latest;
    }
    else
    {
        current.joint_command_latest = current.hold_position;
    }

    if (HasExactPose(previous.topic_command_latest, current.arm_size))
    {
        current.topic_command_latest = previous.topic_command_latest;
        current.topic_command_received = previous.topic_command_received;
    }
    else
    {
        current.topic_command_latest = current.hold_position;
        current.topic_command_received = false;
    }

    if (previous.command_initialized &&
        HasExactPose(previous.command_smoothing_start, current.arm_size) &&
        HasExactPose(previous.command_smoothing_target, current.arm_size) &&
        HasExactPose(previous.command_smoothed, current.arm_size))
    {
        current.command_smoothing_counter = previous.command_smoothing_counter;
        current.command_smoothing_start = previous.command_smoothing_start;
        current.command_smoothing_target = previous.command_smoothing_target;
        current.command_smoothed = previous.command_smoothed;
        current.command_initialized = true;
    }
}

} // namespace Go2X5ControlLogic

#endif // GO2_X5_CONTROL_LOGIC_HPP
