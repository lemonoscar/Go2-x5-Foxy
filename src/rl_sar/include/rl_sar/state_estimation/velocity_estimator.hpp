#ifndef RL_SAR_STATE_ESTIMATION_VELOCITY_ESTIMATOR_HPP
#define RL_SAR_STATE_ESTIMATION_VELOCITY_ESTIMATOR_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <vector>
#include <functional>

namespace rl_sar::state_estimation
{

/**
 * @brief 3D Kalman Filter for velocity estimation
 *
 * Implements a standard Kalman Filter with:
 * - State dimension: 3 (linear velocity x, y, z)
 * - Measurement dimension: 3
 * - Control input dimension: 3 (acceleration)
 */
class KalmanFilter3D
{
public:
    KalmanFilter3D(
        float accelerometer_variance = 0.1f,
        float sensor_variance = 0.1f,
        float initial_variance = 0.1f);

    void Reset();
    void Predict(float dt, const std::array<float, 3>& acceleration);
    void Update(const std::array<float, 3>& measurement);
    void SetState(const std::array<float, 3>& state);
    std::array<float, 3> GetState() const;
    std::array<float, 3> GetCovarianceDiagonal() const;

private:
    std::array<float, 3> x_;                          // State estimate
    std::array<std::array<float, 3>, 3> P_;           // State covariance
    std::array<std::array<float, 3>, 3> Q_;           // Process noise
    std::array<std::array<float, 3>, 3> R_;           // Measurement noise
    float initial_variance_;

    // Helper for matrix multiplication
    std::array<float, 3> MatrixVectorMultiply(
        const std::array<std::array<float, 3>, 3>& M,
        const std::array<float, 3>& v) const;
};

/**
 * @brief Moving window filter for smoothing sensor readings
 *
 * Implements Neumaier's algorithm for numerically stable summation.
 * Based on https://github.com/erwincoumans/motion_imitation
 */
class MovingWindowFilter
{
public:
    explicit MovingWindowFilter(int window_size = 120, int data_dim = 3);

    void Reset();
    std::array<float, 3> CalculateAverage(const std::array<float, 3>& new_value);
    bool IsReady() const;

private:
    int window_size_;
    int data_dim_;
    std::deque<std::array<float, 3>> value_deque_;
    std::array<float, 3> sum_;
    std::array<float, 3> correction_;

    void NeumaierSum(const std::array<float, 3>& value);
    void SubtractOldest(const std::array<float, 3>& value);
};

/**
 * @brief Analytical leg Jacobian computation
 *
 * Computes the Jacobian matrix for a quadruped leg using the analytical method.
 * Based on A1/Go2 leg kinematics.
 */
struct LegJacobianResult
{
    std::array<std::array<float, 3>, 3> jacobian;  // 3x3 Jacobian matrix
    std::array<float, 3> foot_velocity;            // Foot velocity in base frame
};

class LegKinematics
{
public:
    LegKinematics(
        float hip_length = 0.08f,    // Go2 hip abduction
        float thigh_length = 0.213f,  // Go2 thigh
        float calf_length = 0.213f);   // Go2 calf

    /**
     * @brief Compute analytical Jacobian for a leg
     *
     * @param leg_angles 3 joint angles [hip_abduction, hip, knee]
     * @param leg_id Leg index in project order [FR, FL, RR, RL]
     * @return Jacobian matrix (3x3)
     */
    std::array<std::array<float, 3>, 3> AnalyticalLegJacobian(
        const std::array<float, 3>& leg_angles,
        int leg_id) const;

    /**
     * @brief Compute foot velocity from joint velocities
     *
     * @param leg_angles Current joint angles [3]
     * @param joint_velocities Current joint velocities [3]
     * @param leg_id Leg index
     * @return Foot velocity in base frame [3]
     */
    std::array<float, 3> ComputeFootVelocity(
        const std::array<float, 3>& leg_angles,
        const std::array<float, 3>& joint_velocities,
        int leg_id) const;

private:
    float hip_length_;
    float thigh_length_;
    float calf_length_;
};

/**
 * @brief Velocity estimator for quadruped robots
 *
 * Uses a Kalman Filter to fuse:
 * - IMU acceleration (prediction)
 * - Contact leg kinematics (correction)
 *
 * The output is further smoothed by a moving window filter.
 * The public estimate is body-frame linear velocity to match dog-only policy obs.
 *
 * Based on the design from umi-on-legs/real-wbc/modules/velocity_estimator.py
 */
class VelocityEstimator
{
public:
    struct Config
    {
        // Leg kinematics
        float hip_length = 0.08f;
        float thigh_length = 0.213f;
        float calf_length = 0.213f;

        // Kalman filter parameters
        float accelerometer_variance = 0.1f;
        float sensor_variance = 0.1f;
        float initial_variance = 0.1f;

        // Moving window filter
        int moving_window_size = 120;  // ~0.6s at 200Hz

        // Timing
        float default_control_dt = 0.005f;  // 200Hz

        // Foot contact
        float foot_contact_force_threshold = 20.0f;  // Newtons

        // Gravity
        std::array<float, 3> gravity{0.0f, 0.0f, -9.81f};
    };

    explicit VelocityEstimator(const Config& config);

    void Reset();
    void Update(
        uint64_t timestamp_ns,
        const std::array<float, 3>& acceleration,    // IMU acceleration (sensor frame)
        const std::array<float, 3>& gyroscope,       // IMU angular velocity (sensor frame)
        const std::array<float, 4>& quaternion,      // IMU quaternion (w, x, y, z)
        const std::array<float, 12>& joint_position, // 12 joint positions in [FR, FL, RR, RL]
        const std::array<float, 12>& joint_velocity, // 12 joint velocities in [FR, FL, RR, RL]
        const std::array<float, 4>& foot_force);     // 4 foot contact forces in [FR, FL, RR, RL]

    std::array<float, 3> GetEstimatedVelocity() const;
    bool IsReady() const;
    const Config& GetConfig() const { return config_; }

    // Bias calibration
    void SetAccelerometerBias(const std::array<float, 3>& bias);
    void SetVelocityBias(const std::array<float, 3>& bias);

private:
    float ComputeDeltaTime(uint64_t timestamp_ns);
    std::array<float, 3> RotateInverse(
        const std::array<float, 4>& q,
        const std::array<float, 3>& v) const;
    std::array<std::array<float, 3>, 3> QuaternionToRotationMatrix(
        const std::array<float, 4>& q) const;
    std::array<float, 3> MatrixVectorMultiply(
        const std::array<std::array<float, 3>, 3>& M,
        const std::array<float, 3>& v) const;

    Config config_;
    KalmanFilter3D kalman_filter_;
    MovingWindowFilter smoothing_filter_;
    LegKinematics leg_kinematics_;

    std::array<float, 3> estimated_velocity_;
    uint64_t last_timestamp_ns_ = 0;
    bool first_update_ = true;

    // Bias
    std::array<float, 3> accelerometer_bias_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> velocity_bias_{0.0f, 0.0f, 0.0f};
};

} // namespace rl_sar::state_estimation

#endif // RL_SAR_STATE_ESTIMATION_VELOCITY_ESTIMATOR_HPP
