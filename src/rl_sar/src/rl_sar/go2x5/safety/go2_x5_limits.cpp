
#include "rl_real_go2_x5.hpp"
#include <limits>

// ============================================================================
// Default Limits and Bounds
// ============================================================================

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyLowerLimits() const
{
    const int num_dofs = GetNumDofs();
    if (num_dofs == 18)
    {
        return {
            -1.0472f, -1.5708f, -2.7227f,
            -1.0472f, -1.5708f, -2.7227f,
            -1.0472f, -0.5236f, -2.7227f,
            -1.0472f, -0.5236f, -2.7227f,
            -3.14f, -0.05f, -0.1f,
            -1.6f, -1.57f, -2.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, num_dofs)),
                              -std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyUpperLimits() const
{
    const int num_dofs = GetNumDofs();
    if (num_dofs == 18)
    {
        return {
            1.0472f, 3.4907f, -0.83776f,
            1.0472f, 3.4907f, -0.83776f,
            1.0472f, 4.5379f, -0.83776f,
            1.0472f, 4.5379f, -0.83776f,
            2.618f, 3.50f, 3.20f,
            1.55f, 1.57f, 2.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, num_dofs)),
                              std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyVelocityLimits() const
{
    const int num_dofs = GetNumDofs();
    if (num_dofs == 18)
    {
        return {
            30.1f, 30.1f, 15.7f,
            30.1f, 30.1f, 15.7f,
            30.1f, 30.1f, 15.7f,
            30.1f, 30.1f, 15.7f,
            3.0f, 3.0f, 3.0f,
            3.0f, 3.0f, 3.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, num_dofs)),
                              std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyEffortLimits() const
{
    const int num_dofs = GetNumDofs();
    auto torque_limits = config_->GetTorqueLimits();
    if (torque_limits.size() == static_cast<size_t>(std::max(0, num_dofs)))
    {
        return torque_limits;
    }
    if (num_dofs == 18)
    {
        return {
            23.7f, 23.7f, 45.43f,
            23.7f, 23.7f, 45.43f,
            23.7f, 23.7f, 45.43f,
            23.7f, 23.7f, 45.43f,
            15.0f, 15.0f, 15.0f,
            3.0f, 3.0f, 3.0f
        };
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, num_dofs)),
                              std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyKpLimits() const
{
    const int num_dofs = GetNumDofs();
    std::vector<float> limits(static_cast<size_t>(std::max(0, num_dofs)), 0.0f);
    const auto fixed_kp = GetFixedKp();
    const auto rl_kp = config_->GetRlKp();
    for (int i = 0; i < num_dofs; ++i)
    {
        float value = 0.0f;
        if (i < static_cast<int>(fixed_kp.size()))
            value = std::max(value, std::fabs(fixed_kp[static_cast<size_t>(i)]));
        if (i < static_cast<int>(rl_kp.size()))
            value = std::max(value, std::fabs(rl_kp[static_cast<size_t>(i)]));
        limits[static_cast<size_t>(i)] = std::max(1.0f, value);
    }
    return limits;
}

std::vector<float> RL_Real_Go2X5::GetDefaultWholeBodyKdLimits() const
{
    const int num_dofs = GetNumDofs();
    std::vector<float> limits(static_cast<size_t>(std::max(0, num_dofs)), 0.0f);
    const auto fixed_kd = GetFixedKd();
    const auto rl_kd = config_->GetRlKd();
    for (int i = 0; i < num_dofs; ++i)
    {
        float value = 0.0f;
        if (i < static_cast<int>(fixed_kd.size()))
            value = std::max(value, std::fabs(fixed_kd[static_cast<size_t>(i)]));
        if (i < static_cast<int>(rl_kd.size()))
            value = std::max(value, std::fabs(rl_kd[static_cast<size_t>(i)]));
        limits[static_cast<size_t>(i)] = std::max(1.0f, value);
    }
    return limits;
}

std::vector<float> RL_Real_Go2X5::GetDefaultArmLowerLimits() const
{
    const int arm_joint_count = config_->GetArmJointCount();
    if (arm_joint_count == 6)
    {
        return {-3.14f, -0.05f, -0.1f, -1.6f, -1.57f, -2.0f};
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, arm_joint_count)),
                              -std::numeric_limits<float>::infinity());
}

std::vector<float> RL_Real_Go2X5::GetDefaultArmUpperLimits() const
{
    const int arm_joint_count = config_->GetArmJointCount();
    if (arm_joint_count == 6)
    {
        return {2.618f, 3.50f, 3.20f, 1.55f, 1.57f, 2.0f};
    }
    return std::vector<float>(static_cast<size_t>(std::max(0, arm_joint_count)),
                              std::numeric_limits<float>::infinity());
}
