
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/control_logic.hpp"
#include <mutex>

// ============================================================================
// Keyboard and Joystick Control
// ============================================================================

void RL_Real_Go2X5::JoystickHandler(const void *message)
{
    const auto *msg = (const unitree_go::msg::dds_::WirelessController_ *)message;
    if (!msg)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
    this->unitree_joy.value = msg->keys();
    this->unitree_joy_lx = msg->lx();
    this->unitree_joy_ly = msg->ly();
    this->unitree_joy_rx = msg->rx();
}

void RL_Real_Go2X5::HandleKey2ArmHold()
{
    int arm_command_size_local = 0;
    std::vector<float> topic_pose;
    bool topic_received = false;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_command_size_local = this->arm_command_size;
        topic_pose = this->arm_topic_command_latest;
        topic_received = this->arm_topic_command_received;
    }

    const bool key2_prefer_topic_command = config_->GetKey2PreferTopicCommand();
    if (!key2_prefer_topic_command && topic_received)
    {
        std::cout << LOGGER::INFO
                  << "Key[2] ignoring latest " << this->arm_joint_command_topic
                  << " because key2_prefer_topic_command=false." << std::endl;
    }
    const auto key_pose = config_->GetArmKeyPose();
    const auto hold_pose = config_->GetArmHoldPose();
    const auto selected = Go2X5ControlLogic::SelectKey2ArmPose(
        arm_command_size_local,
        key2_prefer_topic_command,
        topic_received,
        topic_pose,
        key_pose,
        hold_pose);

    if (!selected.pose.empty())
    {
        const char* reason = "Key[2] pressed: arm hold pose";
        switch (selected.source)
        {
            case Go2X5ControlLogic::ArmPoseSource::TopicCommand:
                reason = "Key[2] pressed: arm topic command hold";
                break;
            case Go2X5ControlLogic::ArmPoseSource::KeyPose:
                reason = "Key[2] pressed: arm key pose hold";
                break;
            case Go2X5ControlLogic::ArmPoseSource::HoldPose:
                reason = "Key[2] pressed: arm hold pose";
                break;
            case Go2X5ControlLogic::ArmPoseSource::None:
                break;
        }
        this->ApplyArmHold(selected.pose, reason);
    }
    else if (arm_command_size_local > 0)
    {
        std::cout << LOGGER::WARNING
                  << "Key[2] pressed: no valid arm target (topic/key/hold) for arm_command_size="
                  << arm_command_size_local << std::endl;
    }
}

void RL_Real_Go2X5::HandleKey3ArmDefault()
{
    int arm_command_size_local = 0;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_command_size_local = this->arm_command_size;
    }
    const auto default_pos = GetDefaultDofPos();
    const int arm_start = std::max(0, this->arm_joint_start_index);
    if (arm_command_size_local > 0 &&
        default_pos.size() >= static_cast<size_t>(arm_start + arm_command_size_local))
    {
        std::vector<float> pose(
            default_pos.begin() + static_cast<long>(arm_start),
            default_pos.begin() + static_cast<long>(arm_start + arm_command_size_local)
        );
        this->ApplyArmHold(pose, "Key[3] pressed: arm restore default");
    }
}

void RL_Real_Go2X5::HandleKey4ArmHoldToggle()
{
    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    this->arm_hold_enabled = !this->arm_hold_enabled;
    std::cout << LOGGER::INFO << "Key[4] pressed: arm hold "
              << (this->arm_hold_enabled ? "ON" : "OFF") << std::endl;
}
