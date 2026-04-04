
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/safety_guard.hpp"
#include <limits>
#include <algorithm>

// ============================================================================
// Command Clipping and Validation
// ============================================================================

bool RL_Real_Go2X5::ClipArmPoseTargetInPlace(std::vector<float>& target, const std::vector<float>& fallback, const char* context) const
{
    Go2X5SafetyGuard::ArmSafetyContext ctx{this->arm_joint_start_index, this->arm_joint_count,
        this->arm_joint_lower_limits, this->arm_joint_upper_limits, this->whole_body_velocity_limits,
        this->whole_body_effort_limits, this->whole_body_kp_limits, this->whole_body_kd_limits};
    return Go2X5SafetyGuard::ClipArmPoseTargetInPlace(&target, fallback, ctx, context);
}

bool RL_Real_Go2X5::ClipArmBridgeCommandInPlace(std::vector<float>& q, std::vector<float>& dq,
    std::vector<float>& kp, std::vector<float>& kd, std::vector<float>& tau,
    const std::vector<float>& q_fallback, const char* context) const
{
    Go2X5SafetyGuard::ArmSafetyContext ctx{this->arm_joint_start_index, this->arm_joint_count,
        this->arm_joint_lower_limits, this->arm_joint_upper_limits, this->whole_body_velocity_limits,
        this->whole_body_effort_limits, this->whole_body_kp_limits, this->whole_body_kd_limits};
    return Go2X5SafetyGuard::ClipArmBridgeCommandInPlace(&q, &dq, &kp, &kd, &tau, q_fallback, ctx, context);
}

bool RL_Real_Go2X5::ValidateArmPoseTarget(const std::vector<float>& target, const char* context) const
{
    Go2X5SafetyGuard::ArmSafetyContext ctx{this->arm_joint_start_index, this->arm_joint_count,
        this->arm_joint_lower_limits, this->arm_joint_upper_limits, {}, {}, {}, {}};
    return Go2X5SafetyGuard::ValidateArmPoseTarget(target, ctx, context);
}

bool RL_Real_Go2X5::ValidateArmBridgeStateSample(const std::vector<float>& q,
    const std::vector<float>& dq, const std::vector<float>& tau, const char* context) const
{
    Go2X5SafetyGuard::ArmSafetyContext ctx{this->arm_joint_start_index, this->arm_joint_count,
        this->arm_joint_lower_limits, this->arm_joint_upper_limits, {}, {}, {}, {}};
    return Go2X5SafetyGuard::ValidateArmBridgeStateSample(q, dq, tau, ctx, context);
}

void RL_Real_Go2X5::ValidateJointMappingOrThrow(const char* stage) const
{
    const int num_dofs = GetNumDofs();
    if (num_dofs <= 0)
    {
        throw std::runtime_error(std::string("Invalid num_of_dofs in ") + stage);
    }

    const auto joint_mapping = GetJointMapping();
    if (joint_mapping.size() != static_cast<size_t>(num_dofs))
    {
        throw std::runtime_error(
            std::string("joint_mapping size mismatch in ") + stage +
            ": expect " + std::to_string(num_dofs) +
            ", got " + std::to_string(joint_mapping.size()));
    }

    std::vector<bool> used(20, false);
    for (size_t i = 0; i < joint_mapping.size(); ++i)
    {
        const int mapped = joint_mapping[i];
        if (mapped < 0 || mapped >= 20)
        {
            throw std::runtime_error(
                std::string("joint_mapping out of range in ") + stage +
                " at index " + std::to_string(i) +
                ", value=" + std::to_string(mapped) + ", expected [0,19]");
        }
        if (used[static_cast<size_t>(mapped)])
        {
            std::cout << "[WARNING] Duplicate joint_mapping value " << static_cast<int>(mapped)
                      << " detected in " << stage << "." << std::endl;
        }
        used[static_cast<size_t>(mapped)] = true;
    }
}
