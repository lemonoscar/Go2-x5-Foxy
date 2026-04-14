#ifndef RL_SAR_OBSERVATION_OBSERVATION_BUILDER_HPP
#define RL_SAR_OBSERVATION_OBSERVATION_BUILDER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rl_sar::observation
{

class ObservationBuilder
{
public:
    static constexpr size_t kObservationDim = 260;
    static constexpr size_t kCommandDim = 3;
    static constexpr size_t kJointDim = 18;
    static constexpr size_t kArmJointDim = 6;
    static constexpr size_t kHeightScanDim = 187;
    static constexpr uint32_t kSnapshotMagic = 0x3142534fU;  // "OSB1"
    static constexpr uint32_t kSnapshotVersion = 1U;

    struct Config
    {
        float clip_obs = 100.0f;
        float lin_vel_scale = 1.0f;
        float ang_vel_scale = 1.0f;
        float dof_pos_scale = 1.0f;
        float dof_vel_scale = 1.0f;
        std::array<float, kCommandDim> commands_scale{{1.0f, 1.0f, 1.0f}};
        std::array<float, kJointDim> default_dof_pos{};
        std::array<bool, kJointDim> zero_dof_pos_indices{};
        int actions_observation_dim = static_cast<int>(kJointDim);
        float gripper_command_default = 0.0f;
        std::string ang_vel_axis = "body";
    };

    struct BuildInput
    {
        std::array<float, kCommandDim> base_lin_vel{};
        std::array<float, kCommandDim> base_ang_vel{};
        std::array<float, 4> base_quat{{1.0f, 0.0f, 0.0f, 0.0f}};
        std::array<float, kCommandDim> gravity_vec{{0.0f, 0.0f, -1.0f}};
        std::array<float, kCommandDim> velocity_commands{};
        std::array<float, kJointDim> full_joint_pos{};
        std::array<float, kJointDim> full_joint_vel{};
        std::array<float, kJointDim> last_actions{};
        std::array<float, kHeightScanDim> height_scan{};
        std::array<float, kArmJointDim> arm_joint_command{};
        float gripper_command = 0.0f;
    };

    struct SnapshotFile
    {
        uint32_t magic = kSnapshotMagic;
        uint32_t version = kSnapshotVersion;
        BuildInput input{};
        std::array<float, kObservationDim> reference_obs{};
        std::array<char, 128> description{};
    };

    ObservationBuilder();
    explicit ObservationBuilder(const Config& config);

    const Config& GetConfig() const;

    std::array<float, kObservationDim> Build(const BuildInput& input) const;

    bool ValidateAgainstSnapshot(const std::string& snapshot_path, std::string* error = nullptr) const;

    void PrintStatistics(const std::array<float, kObservationDim>& obs) const;

    static bool HasStandardLayout(const std::vector<std::string>& observations);

private:
    static std::array<float, kCommandDim> RotateInverse(
        const std::array<float, 4>& quat,
        const std::array<float, kCommandDim>& vec);

    static void PrintSegmentStatistics(
        const char* label,
        const std::array<float, kObservationDim>& obs,
        size_t begin,
        size_t end);

    float ClampValue(float value) const;

    Config config_;
};

}  // namespace rl_sar::observation

#endif  // RL_SAR_OBSERVATION_OBSERVATION_BUILDER_HPP
