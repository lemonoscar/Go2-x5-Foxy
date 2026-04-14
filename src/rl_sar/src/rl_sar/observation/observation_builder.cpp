#include "rl_sar/observation/observation_builder.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace rl_sar::observation
{

namespace
{

constexpr std::array<const char*, 10> kStandardObservationLayout{{
    "lin_vel",
    "ang_vel",
    "gravity_vec",
    "commands",
    "dof_pos",
    "dof_vel",
    "actions",
    "height_scan",
    "arm_joint_command",
    "gripper_command",
}};

}  // namespace

ObservationBuilder::ObservationBuilder() = default;

ObservationBuilder::ObservationBuilder(const Config& config)
    : config_(config)
{
}

const ObservationBuilder::Config& ObservationBuilder::GetConfig() const
{
    return config_;
}

std::array<float, ObservationBuilder::kObservationDim> ObservationBuilder::Build(
    const BuildInput& input) const
{
    std::array<float, kObservationDim> obs{};
    size_t idx = 0;

    const auto ang_vel =
        config_.ang_vel_axis == "world"
            ? RotateInverse(input.base_quat, input.base_ang_vel)
            : input.base_ang_vel;
    const auto projected_gravity = RotateInverse(input.base_quat, input.gravity_vec);

    for (float value : input.base_lin_vel)
    {
        obs[idx++] = value * config_.lin_vel_scale;
    }
    for (float value : ang_vel)
    {
        obs[idx++] = value * config_.ang_vel_scale;
    }
    for (float value : projected_gravity)
    {
        obs[idx++] = value;
    }
    for (size_t i = 0; i < kCommandDim; ++i)
    {
        obs[idx++] = input.velocity_commands[i] * config_.commands_scale[i];
    }
    for (size_t i = 0; i < kJointDim; ++i)
    {
        float dof_pos = (input.full_joint_pos[i] - config_.default_dof_pos[i]) * config_.dof_pos_scale;
        if (config_.zero_dof_pos_indices[i])
        {
            dof_pos = 0.0f;
        }
        obs[idx++] = dof_pos;
    }
    for (float value : input.full_joint_vel)
    {
        obs[idx++] = value * config_.dof_vel_scale;
    }
    for (float value : input.last_actions)
    {
        obs[idx++] = value;
    }
    for (float value : input.height_scan)
    {
        obs[idx++] = value;
    }
    for (float value : input.arm_joint_command)
    {
        obs[idx++] = value;
    }
    obs[idx++] = input.gripper_command;

    for (float& value : obs)
    {
        value = ClampValue(value);
    }
    return obs;
}

bool ObservationBuilder::ValidateAgainstSnapshot(
    const std::string& snapshot_path,
    std::string* error) const
{
    std::ifstream input(snapshot_path, std::ios::binary);
    if (!input.is_open())
    {
        if (error)
        {
            *error = "failed to open snapshot file: " + snapshot_path;
        }
        return false;
    }

    SnapshotFile snapshot;
    input.read(reinterpret_cast<char*>(&snapshot), sizeof(snapshot));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(snapshot)))
    {
        if (error)
        {
            *error = "unexpected snapshot size in " + snapshot_path;
        }
        return false;
    }
    if (snapshot.magic != kSnapshotMagic || snapshot.version != kSnapshotVersion)
    {
        if (error)
        {
            *error = "unsupported snapshot header in " + snapshot_path;
        }
        return false;
    }

    const auto actual = Build(snapshot.input);
    for (size_t i = 0; i < kObservationDim; ++i)
    {
        const float diff = std::fabs(actual[i] - snapshot.reference_obs[i]);
        if (diff > 1e-4f)
        {
            if (error)
            {
                *error = "snapshot mismatch at dim " + std::to_string(i) +
                    ": actual=" + std::to_string(actual[i]) +
                    " reference=" + std::to_string(snapshot.reference_obs[i]);
            }
            return false;
        }
    }

    return true;
}

void ObservationBuilder::PrintStatistics(const std::array<float, kObservationDim>& obs) const
{
    std::cout << "[ObservationBuilder] ===== Observation Statistics =====" << std::endl;
    PrintSegmentStatistics("base_lin_vel", obs, 0, 3);
    PrintSegmentStatistics("base_ang_vel", obs, 3, 6);
    PrintSegmentStatistics("projected_gravity", obs, 6, 9);
    PrintSegmentStatistics("velocity_commands", obs, 9, 12);
    PrintSegmentStatistics("joint_pos", obs, 12, 30);
    PrintSegmentStatistics("joint_vel", obs, 30, 48);
    PrintSegmentStatistics("last_actions", obs, 48, 66);
    PrintSegmentStatistics("height_scan", obs, 66, 253);
    PrintSegmentStatistics("arm_joint_command", obs, 253, 259);
    PrintSegmentStatistics("gripper_command", obs, 259, 260);
    std::cout << "[ObservationBuilder] ===== End Statistics =====" << std::endl;
}

bool ObservationBuilder::HasStandardLayout(const std::vector<std::string>& observations)
{
    if (observations.size() != kStandardObservationLayout.size())
    {
        return false;
    }
    for (size_t i = 0; i < observations.size(); ++i)
    {
        if (observations[i] != kStandardObservationLayout[i])
        {
            return false;
        }
    }
    return true;
}

std::array<float, ObservationBuilder::kCommandDim> ObservationBuilder::RotateInverse(
    const std::array<float, 4>& quat,
    const std::array<float, kCommandDim>& vec)
{
    const float q_w = quat[0];
    const float q_x = quat[1];
    const float q_y = quat[2];
    const float q_z = quat[3];

    const float v_x = vec[0];
    const float v_y = vec[1];
    const float v_z = vec[2];

    const float a_x = v_x * (2.0f * q_w * q_w - 1.0f);
    const float a_y = v_y * (2.0f * q_w * q_w - 1.0f);
    const float a_z = v_z * (2.0f * q_w * q_w - 1.0f);

    const float cross_x = q_y * v_z - q_z * v_y;
    const float cross_y = q_z * v_x - q_x * v_z;
    const float cross_z = q_x * v_y - q_y * v_x;

    const float b_x = cross_x * q_w * 2.0f;
    const float b_y = cross_y * q_w * 2.0f;
    const float b_z = cross_z * q_w * 2.0f;

    const float dot = q_x * v_x + q_y * v_y + q_z * v_z;

    const float c_x = q_x * dot * 2.0f;
    const float c_y = q_y * dot * 2.0f;
    const float c_z = q_z * dot * 2.0f;

    return {a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z};
}

void ObservationBuilder::PrintSegmentStatistics(
    const char* label,
    const std::array<float, kObservationDim>& obs,
    const size_t begin,
    const size_t end)
{
    float min_value = std::numeric_limits<float>::infinity();
    float max_value = -std::numeric_limits<float>::infinity();
    double sum = 0.0;
    for (size_t i = begin; i < end; ++i)
    {
        min_value = std::min(min_value, obs[i]);
        max_value = std::max(max_value, obs[i]);
        sum += obs[i];
    }
    const double mean = (end > begin) ? (sum / static_cast<double>(end - begin)) : 0.0;
    std::cout << "[ObservationBuilder] dim " << begin << "-" << (end - 1)
              << " (" << label << ")"
              << " min=" << std::fixed << std::setprecision(6) << min_value
              << " max=" << max_value
              << " mean=" << mean
              << std::endl;
}

float ObservationBuilder::ClampValue(const float value) const
{
    return std::max(-config_.clip_obs, std::min(config_.clip_obs, value));
}

}  // namespace rl_sar::observation
