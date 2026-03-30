#ifndef GO2_X5_SAFETY_GUARD_HPP
#define GO2_X5_SAFETY_GUARD_HPP

#include <chrono>
#include <vector>

namespace Go2X5SafetyGuard
{

struct ArmSafetyContext
{
    int arm_joint_start_index = 0;
    int arm_joint_count = 0;
    std::vector<float> arm_joint_lower_limits;
    std::vector<float> arm_joint_upper_limits;
    std::vector<float> whole_body_velocity_limits;
    std::vector<float> whole_body_effort_limits;
    std::vector<float> whole_body_kp_limits;
    std::vector<float> whole_body_kd_limits;
};

struct ArmBridgeStateSnapshot
{
    bool state_valid = false;
    bool require_live_state = true;
    bool state_from_backend = false;
    float state_timeout_sec = 0.0f;
    std::chrono::steady_clock::time_point state_stamp{};
};

bool ClipArmPoseTargetInPlace(std::vector<float>* target,
                              const std::vector<float>& fallback,
                              const ArmSafetyContext& safety_context,
                              const char* context);

bool ClipArmBridgeCommandInPlace(std::vector<float>* q,
                                 std::vector<float>* dq,
                                 std::vector<float>* kp,
                                 std::vector<float>* kd,
                                 std::vector<float>* tau,
                                 const std::vector<float>& q_fallback,
                                 const ArmSafetyContext& safety_context,
                                 const char* context);

bool ValidateArmPoseTarget(const std::vector<float>& target,
                           const ArmSafetyContext& safety_context,
                           const char* context);

bool ValidateArmBridgeStateSample(const std::vector<float>& q,
                                  const std::vector<float>& dq,
                                  const std::vector<float>& tau,
                                  const ArmSafetyContext& safety_context,
                                  const char* context);

bool IsArmBridgeStateFresh(const ArmBridgeStateSnapshot& state_snapshot,
                           std::chrono::steady_clock::time_point now = std::chrono::steady_clock::time_point{});

} // namespace Go2X5SafetyGuard

#endif // GO2_X5_SAFETY_GUARD_HPP
