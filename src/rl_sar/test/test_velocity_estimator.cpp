#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "rl_sar/state_estimation/velocity_estimator.hpp"

namespace rl_sar::state_estimation::test
{

namespace
{

std::array<float, 12> MakeStandingJointPosition()
{
    return {
        0.0f, 0.8f, -1.5f,
        0.0f, 0.8f, -1.5f,
        0.0f, 0.8f, -1.5f,
        0.0f, 0.8f, -1.5f,
    };
}

std::array<float, 12> MakeZeroJointVelocity()
{
    std::array<float, 12> joint_velocity{};
    joint_velocity.fill(0.0f);
    return joint_velocity;
}

std::array<float, 4> MakeYawQuaternion(const float yaw_rad)
{
    return {
        std::cos(yaw_rad * 0.5f),
        0.0f,
        0.0f,
        std::sin(yaw_rad * 0.5f),
    };
}

}  // namespace

TEST(VelocityEstimatorTest, StationaryRobotKeepsVelocityNearZero)
{
    VelocityEstimator::Config config;
    config.moving_window_size = 4;

    VelocityEstimator estimator(config);
    const auto joint_position = MakeStandingJointPosition();
    const auto joint_velocity = MakeZeroJointVelocity();
    const std::array<float, 4> foot_force{50.0f, 50.0f, 50.0f, 50.0f};
    const std::array<float, 4> quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    const std::array<float, 3> gyroscope{0.0f, 0.0f, 0.0f};
    const std::array<float, 3> acceleration{0.0f, 0.0f, 9.81f};

    for (uint64_t step = 0; step < 8; ++step)
    {
        estimator.Update(
            (step + 1) * 5000000ULL,
            acceleration,
            gyroscope,
            quaternion,
            joint_position,
            joint_velocity,
            foot_force);
    }

    const auto estimated_velocity = estimator.GetEstimatedVelocity();
    EXPECT_TRUE(estimator.IsReady());
    EXPECT_NEAR(estimated_velocity[0], 0.0f, 1e-3f);
    EXPECT_NEAR(estimated_velocity[1], 0.0f, 1e-3f);
    EXPECT_NEAR(estimated_velocity[2], 0.0f, 1e-3f);
}

TEST(VelocityEstimatorTest, ReturnsBodyFrameVelocityForYawedBase)
{
    VelocityEstimator::Config config;
    config.accelerometer_variance = 1e-6f;
    config.sensor_variance = 1e-6f;
    config.initial_variance = 1.0f;
    config.moving_window_size = 1;

    VelocityEstimator estimator(config);
    LegKinematics kinematics(config.hip_length, config.thigh_length, config.calf_length);

    auto joint_position = MakeStandingJointPosition();
    auto joint_velocity = MakeZeroJointVelocity();
    joint_velocity[0] = 0.3f;
    joint_velocity[1] = -0.4f;
    joint_velocity[2] = 0.2f;

    const std::array<float, 4> foot_force{50.0f, 0.0f, 0.0f, 0.0f};
    const std::array<float, 4> quaternion = MakeYawQuaternion(1.57079632679f);
    const std::array<float, 3> gyroscope{0.0f, 0.0f, 0.0f};
    const std::array<float, 3> acceleration{0.0f, 0.0f, 9.81f};

    const std::array<float, 3> expected_body_velocity = kinematics.ComputeFootVelocity(
        {joint_position[0], joint_position[1], joint_position[2]},
        {joint_velocity[0], joint_velocity[1], joint_velocity[2]},
        0);

    for (uint64_t step = 0; step < 3; ++step)
    {
        estimator.Update(
            (step + 1) * 5000000ULL,
            acceleration,
            gyroscope,
            quaternion,
            joint_position,
            joint_velocity,
            foot_force);
    }

    const auto estimated_velocity = estimator.GetEstimatedVelocity();
    EXPECT_TRUE(estimator.IsReady());
    EXPECT_GT(std::fabs(expected_body_velocity[0]) +
              std::fabs(expected_body_velocity[1]) +
              std::fabs(expected_body_velocity[2]), 1e-4f);
    EXPECT_NEAR(estimated_velocity[0], expected_body_velocity[0], 1e-3f);
    EXPECT_NEAR(estimated_velocity[1], expected_body_velocity[1], 1e-3f);
    EXPECT_NEAR(estimated_velocity[2], expected_body_velocity[2], 1e-3f);
}

}  // namespace rl_sar::state_estimation::test
