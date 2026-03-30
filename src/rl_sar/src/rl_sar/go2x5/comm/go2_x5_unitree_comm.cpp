
#include "rl_real_go2_x5.hpp"
#include <mutex>
#include <iostream>

// ============================================================================
// Unitree Communication Functions
// ============================================================================

void RL_Real_Go2X5::InitLowCmd()
{
    this->unitree_low_command.head()[0] = 0xFE;
    this->unitree_low_command.head()[1] = 0xEF;
    this->unitree_low_command.level_flag() = 0xFF;
    this->unitree_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = (0x01);
        this->unitree_low_command.motor_cmd()[i].q() = (PosStopF);
        this->unitree_low_command.motor_cmd()[i].kp() = (0);
        this->unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
        this->unitree_low_command.motor_cmd()[i].kd() = (0);
        this->unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

int RL_Real_Go2X5::QueryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = this->msc.CheckMode(robotForm, motionName);
    if (ret == 0)
    {
        std::cout << "CheckMode succeeded." << std::endl;
    }
    else
    {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if (motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = QueryServiceName(robotForm, motionName);
        std::cout << "Service: " << serviceName << " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string RL_Real_Go2X5::QueryServiceName(std::string form, std::string name)
{
    if (form == "0")
    {
        if (name == "normal") return "sport_mode";
        if (name == "ai") return "ai_sport";
        if (name == "advanced") return "advanced_sport";
    }
    else
    {
        if (name == "ai-w") return "wheeled_sport(go2W)";
        if (name == "normal-w") return "wheeled_sport(b2W)";
    }
    return "";
}

void RL_Real_Go2X5::LowStateMessageHandler(const void *message)
{
    const auto *msg = (const unitree_go::msg::dds_::LowState_ *)message;
    if (!msg)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
    for (size_t i = 0; i < this->unitree_imu_quaternion.size(); ++i)
    {
        this->unitree_imu_quaternion[i] = msg->imu_state().quaternion()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < this->unitree_imu_gyroscope.size(); ++i)
    {
        this->unitree_imu_gyroscope[i] = msg->imu_state().gyroscope()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < this->unitree_motor_q.size(); ++i)
    {
        const auto &m = msg->motor_state()[static_cast<int>(i)];
        this->unitree_motor_q[i] = m.q();
        this->unitree_motor_dq[i] = m.dq();
        this->unitree_motor_tau[i] = m.tau_est();
    }
}
