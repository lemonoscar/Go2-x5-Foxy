
#ifndef GO2_X5_SAFE_SHUTDOWN_HPP
#define GO2_X5_SAFE_SHUTDOWN_HPP

#include <vector>
#include <chrono>

namespace Go2X5SafeShutdown {

struct ShutdownConfig
{
    float shutdown_soft_land_sec;
    float shutdown_hold_sec;
    std::vector<float> shutdown_lie_pose;
    std::vector<float> arm_shutdown_pose;
    std::vector<float> arm_hold_pose;
};

std::vector<float> BuildSafeShutdownTargetPose(
    int num_dofs,
    int arm_joint_count,
    int arm_joint_start_index,
    const ShutdownConfig& config,
    const std::vector<float>& default_pos);

void ExecuteSmoothTransition(
    const std::vector<float>& start_pose,
    const std::vector<float>& target_pose,
    const std::vector<float>& kp,
    const std::vector<float>& kd,
    float soft_land_sec,
    float hold_sec,
    std::function<void(const std::vector<float>&, const std::vector<float>&)> publish_fn);

} // namespace Go2X5SafeShutdown

#endif // GO2_X5_SAFE_SHUTDOWN_HPP
