#include <array>
#include <filesystem>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>

#include "rl_sdk.hpp"
#include "rl_sar/observation/observation_builder.hpp"

namespace rl_sar::observation::test
{

namespace
{

class TestRL final : public RL
{
public:
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

std::vector<float> MakeRampVector(const size_t count, const float start, const float step)
{
    std::vector<float> values;
    values.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        values.push_back(start + static_cast<float>(i) * step);
    }
    return values;
}

template <size_t N>
std::array<float, N> VectorToArray(const std::vector<float>& values)
{
    std::array<float, N> out{};
    for (size_t i = 0; i < N && i < values.size(); ++i)
    {
        out[i] = values[i];
    }
    return out;
}

ObservationBuilder::Config MakeConfig()
{
    ObservationBuilder::Config config;
    config.clip_obs = 100.0f;
    config.lin_vel_scale = 2.0f;
    config.ang_vel_scale = 0.5f;
    config.dof_pos_scale = 3.0f;
    config.dof_vel_scale = 0.25f;
    config.commands_scale = {1.5f, 2.5f, 3.5f};
    config.ang_vel_axis = "world";
    config.gripper_command_default = 0.7f;
    config.actions_observation_dim = 18;
    for (size_t i = 0; i < ObservationBuilder::kJointDim; ++i)
    {
        config.default_dof_pos[i] = -0.3f + static_cast<float>(i) * 0.05f;
    }
    config.zero_dof_pos_indices[1] = true;
    config.zero_dof_pos_indices[13] = true;
    return config;
}

YAML::Node MakeParamsNode(const ObservationBuilder::Config& config)
{
    YAML::Node node;
    node["observations"] = std::vector<std::string>{
        "lin_vel", "ang_vel", "gravity_vec", "commands", "dof_pos",
        "dof_vel", "actions", "height_scan", "arm_joint_command", "gripper_command"};
    node["lin_vel_scale"] = config.lin_vel_scale;
    node["ang_vel_scale"] = config.ang_vel_scale;
    node["dof_pos_scale"] = config.dof_pos_scale;
    node["dof_vel_scale"] = config.dof_vel_scale;
    node["commands_scale"] = std::vector<float>{
        config.commands_scale[0], config.commands_scale[1], config.commands_scale[2]};
    node["default_dof_pos"] = std::vector<float>(
        config.default_dof_pos.begin(), config.default_dof_pos.end());
    node["wheel_indices"] = std::vector<int>{1, 13};
    node["actions_observation_dim"] = config.actions_observation_dim;
    node["height_scan_size"] = static_cast<int>(ObservationBuilder::kHeightScanDim);
    node["arm_command_size"] = static_cast<int>(ObservationBuilder::kArmJointDim);
    node["num_of_dofs"] = static_cast<int>(ObservationBuilder::kJointDim);
    node["clip_obs"] = config.clip_obs;
    node["gripper_command_default"] = config.gripper_command_default;
    return node;
}

ObservationBuilder::BuildInput MakeBuildInput()
{
    ObservationBuilder::BuildInput input;
    input.base_lin_vel = {0.3f, -0.4f, 0.5f};
    input.base_ang_vel = {1.2f, -0.7f, 0.9f};
    input.base_quat = {0.9238795f, 0.0f, 0.3826834f, 0.0f};
    input.gravity_vec = {0.0f, 0.0f, -1.0f};
    input.velocity_commands = {0.15f, -0.25f, 0.35f};
    input.full_joint_pos = VectorToArray<ObservationBuilder::kJointDim>(
        MakeRampVector(ObservationBuilder::kJointDim, -0.2f, 0.05f));
    input.full_joint_vel = VectorToArray<ObservationBuilder::kJointDim>(
        MakeRampVector(ObservationBuilder::kJointDim, 0.1f, -0.02f));
    input.last_actions = VectorToArray<ObservationBuilder::kJointDim>(
        MakeRampVector(ObservationBuilder::kJointDim, -0.3f, 0.03f));
    input.height_scan = VectorToArray<ObservationBuilder::kHeightScanDim>(
        MakeRampVector(ObservationBuilder::kHeightScanDim, -0.5f, 0.01f));
    input.arm_joint_command = VectorToArray<ObservationBuilder::kArmJointDim>(
        MakeRampVector(ObservationBuilder::kArmJointDim, 0.4f, 0.02f));
    input.gripper_command = 0.7f;
    return input;
}

std::unique_ptr<TestRL> MakeReferenceRL(
    const ObservationBuilder::Config& config,
    const ObservationBuilder::BuildInput& input)
{
    auto rl = std::make_unique<TestRL>();
    rl->params.config_node = MakeParamsNode(config);
    rl->ang_vel_axis = config.ang_vel_axis;
    rl->obs.lin_vel = std::vector<float>(input.base_lin_vel.begin(), input.base_lin_vel.end());
    rl->obs.ang_vel = std::vector<float>(input.base_ang_vel.begin(), input.base_ang_vel.end());
    rl->obs.base_quat = std::vector<float>(input.base_quat.begin(), input.base_quat.end());
    rl->obs.gravity_vec = std::vector<float>(input.gravity_vec.begin(), input.gravity_vec.end());
    rl->obs.commands = std::vector<float>(input.velocity_commands.begin(), input.velocity_commands.end());
    rl->obs.dof_pos = std::vector<float>(input.full_joint_pos.begin(), input.full_joint_pos.end());
    rl->obs.dof_vel = std::vector<float>(input.full_joint_vel.begin(), input.full_joint_vel.end());
    rl->obs.actions = std::vector<float>(input.last_actions.begin(), input.last_actions.end());
    rl->obs.height_scan = std::vector<float>(input.height_scan.begin(), input.height_scan.end());
    rl->obs.arm_joint_command =
        std::vector<float>(input.arm_joint_command.begin(), input.arm_joint_command.end());
    return rl;
}

}  // namespace

TEST(ObservationBuilderTest, MatchesLegacyComputeObservationForStandardLayout)
{
    const auto config = MakeConfig();
    const auto input = MakeBuildInput();
    ObservationBuilder builder(config);
    const auto built = builder.Build(input);

    auto rl = MakeReferenceRL(config, input);
    const auto expected = rl->ComputeObservation();

    ASSERT_EQ(expected.size(), ObservationBuilder::kObservationDim);
    for (size_t i = 0; i < ObservationBuilder::kObservationDim; ++i)
    {
        EXPECT_NEAR(built[i], expected[i], 1e-5f) << "dim=" << i;
    }
}

TEST(ObservationBuilderTest, ValidatesSnapshotBinary)
{
    const auto config = MakeConfig();
    const auto input = MakeBuildInput();
    ObservationBuilder builder(config);
    ObservationBuilder::SnapshotFile snapshot;
    snapshot.input = input;
    snapshot.reference_obs = builder.Build(input);

    const auto temp_path = std::filesystem::temp_directory_path() / "observation_builder_snapshot.bin";
    {
        std::ofstream output(temp_path, std::ios::binary);
        ASSERT_TRUE(output.is_open());
        output.write(reinterpret_cast<const char*>(&snapshot), sizeof(snapshot));
    }

    std::string error;
    EXPECT_TRUE(builder.ValidateAgainstSnapshot(temp_path.string(), &error)) << error;

    snapshot.reference_obs[9] += 0.25f;
    {
        std::ofstream output(temp_path, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output.write(reinterpret_cast<const char*>(&snapshot), sizeof(snapshot));
    }

    EXPECT_FALSE(builder.ValidateAgainstSnapshot(temp_path.string(), &error));
    std::filesystem::remove(temp_path);
}

TEST(ObservationBuilderTest, DetectsStandardObservationLayout)
{
    EXPECT_TRUE(ObservationBuilder::HasStandardLayout({
        "lin_vel", "ang_vel", "gravity_vec", "commands", "dof_pos",
        "dof_vel", "actions", "height_scan", "arm_joint_command", "gripper_command"}));
    EXPECT_FALSE(ObservationBuilder::HasStandardLayout({
        "lin_vel", "commands", "ang_vel"}));
}

TEST(ObservationBuilderTest, StatisticsPrintingDoesNotThrow)
{
    ObservationBuilder builder(MakeConfig());
    const auto obs = builder.Build(MakeBuildInput());
    EXPECT_NO_THROW(builder.PrintStatistics(obs));
}

}  // namespace rl_sar::observation::test
