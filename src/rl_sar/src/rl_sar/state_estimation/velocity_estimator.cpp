#include "rl_sar/state_estimation/velocity_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace rl_sar::state_estimation
{

// ============================================================================
// KalmanFilter3D Implementation
// ============================================================================

std::array<float, 3> KalmanFilter3D::MatrixVectorMultiply(
    const std::array<std::array<float, 3>, 3>& M,
    const std::array<float, 3>& v) const
{
    return {
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
    };
}

KalmanFilter3D::KalmanFilter3D(
    float accelerometer_variance,
    float sensor_variance,
    float initial_variance)
    : initial_variance_(initial_variance)
{
    // Initialize state to zero
    x_.fill(0.0f);

    // Initialize covariance as diagonal
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            P_[i][j] = (i == j) ? initial_variance : 0.0f;
            Q_[i][j] = (i == j) ? accelerometer_variance : 0.0f;
            R_[i][j] = (i == j) ? sensor_variance : 0.0f;
        }
    }
}

void KalmanFilter3D::Reset()
{
    x_.fill(0.0f);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            P_[i][j] = (i == j) ? initial_variance_ : 0.0f;
        }
    }
}

void KalmanFilter3D::Predict(
    float dt,
    const std::array<float, 3>& acceleration)
{
    // x = x + a * dt
    for (int i = 0; i < 3; ++i)
    {
        x_[i] = x_[i] + acceleration[i] * dt;
    }

    // Match the original filterpy-based umi estimator: Q is applied per step.
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            P_[i][j] += Q_[i][j];
        }
    }
}

void KalmanFilter3D::Update(const std::array<float, 3>& measurement)
{
    // y = z - H * x (innovation)
    std::array<float, 3> y;
    for (int i = 0; i < 3; ++i)
    {
        y[i] = measurement[i] - x_[i];
    }

    // S = H * P * H^T + R = P + R (since H = I)
    std::array<std::array<float, 3>, 3> S;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            S[i][j] = P_[i][j] + R_[i][j];
        }
    }

    // Compute S inverse (3x3)
    float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
                - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
                + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    if (std::abs(det) < 1e-6f)
    {
        return;  // Skip update if singular
    }

    float inv_det = 1.0f / det;
    std::array<std::array<float, 3>, 3> S_inv;
    S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * inv_det;
    S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * inv_det;
    S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv_det;
    S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * inv_det;
    S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv_det;
    S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * inv_det;
    S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * inv_det;
    S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * inv_det;
    S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv_det;

    // K = P * S^-1
    std::array<std::array<float, 3>, 3> K;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                K[i][j] += P_[i][k] * S_inv[k][j];
            }
        }
    }

    // x = x + K * y
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            x_[i] += K[i][j] * y[j];
        }
    }

    // Joseph-form covariance update for numerical stability:
    // P = (I - K) P (I - K)^T + K R K^T
    const auto P_old = P_;
    std::array<std::array<float, 3>, 3> I_minus_K{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            I_minus_K[i][j] = ((i == j) ? 1.0f : 0.0f) - K[i][j];
        }
    }

    std::array<std::array<float, 3>, 3> temp{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            temp[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                temp[i][j] += I_minus_K[i][k] * P_old[k][j];
            }
        }
    }

    std::array<std::array<float, 3>, 3> joseph_left{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            joseph_left[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                joseph_left[i][j] += temp[i][k] * I_minus_K[j][k];
            }
        }
    }

    std::array<std::array<float, 3>, 3> joseph_right{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            joseph_right[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                joseph_right[i][j] += K[i][k] * R_[k][k] * K[j][k];
            }
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            P_[i][j] = joseph_left[i][j] + joseph_right[i][j];
        }
    }
}

void KalmanFilter3D::SetState(const std::array<float, 3>& state)
{
    x_ = state;
}

std::array<float, 3> KalmanFilter3D::GetState() const
{
    return x_;
}

std::array<float, 3> KalmanFilter3D::GetCovarianceDiagonal() const
{
    return {P_[0][0], P_[1][1], P_[2][2]};
}

// ============================================================================
// MovingWindowFilter Implementation
// ============================================================================

MovingWindowFilter::MovingWindowFilter(int window_size, int data_dim)
    : window_size_(window_size)
    , data_dim_(data_dim)
{
    sum_.fill(0.0f);
    correction_.fill(0.0f);
}

void MovingWindowFilter::Reset()
{
    value_deque_.clear();
    sum_.fill(0.0f);
    correction_.fill(0.0f);
}

std::array<float, 3> MovingWindowFilter::CalculateAverage(const std::array<float, 3>& new_value)
{
    if (value_deque_.size() >= static_cast<size_t>(window_size_))
    {
        // Remove oldest value
        SubtractOldest(value_deque_.front());
        value_deque_.pop_front();
    }

    // Add new value
    NeumaierSum(new_value);
    value_deque_.push_back(new_value);

    int count = static_cast<int>(value_deque_.size());
    float scale = 1.0f / count;

    std::array<float, 3> result;
    for (int i = 0; i < 3; ++i)
    {
        result[i] = (sum_[i] + correction_[i]) * scale;
    }
    return result;
}

bool MovingWindowFilter::IsReady() const
{
    return value_deque_.size() >= static_cast<size_t>(window_size_);
}

void MovingWindowFilter::NeumaierSum(const std::array<float, 3>& value)
{
    for (int k = 0; k < 3; ++k)
    {
        float new_sum = sum_[k] + value[k];
        if (std::abs(sum_[k]) >= std::abs(value[k]))
        {
            correction_[k] += (sum_[k] - new_sum) + value[k];
        }
        else
        {
            correction_[k] += (value[k] - new_sum) + sum_[k];
        }
        sum_[k] = new_sum;
    }
}

void MovingWindowFilter::SubtractOldest(const std::array<float, 3>& value)
{
    NeumaierSum({-value[0], -value[1], -value[2]});
}

// ============================================================================
// LegKinematics Implementation
// ============================================================================

LegKinematics::LegKinematics(float hip_length, float thigh_length, float calf_length)
    : hip_length_(hip_length)
    , thigh_length_(thigh_length)
    , calf_length_(calf_length)
{
}

std::array<std::array<float, 3>, 3> LegKinematics::AnalyticalLegJacobian(
    const std::array<float, 3>& leg_angles,
    int leg_id) const
{
    (void)leg_id;

    // Match umi-on-legs analytical Jacobian exactly. The left/right sign is already
    // encoded in the physical hip joint angle, so no extra leg-dependent sign is applied.
    const float hip_angle = leg_angles[0];
    const float thigh_angle = leg_angles[1];
    const float calf_angle = leg_angles[2];

    // Compute effective leg length
    float leg_length_eff = std::sqrt(
        thigh_length_ * thigh_length_ +
        calf_length_ * calf_length_ +
        2.0f * thigh_length_ * calf_length_ * std::cos(calf_angle));

    float leg_angle_eff = thigh_angle + calf_angle / 2.0f;

    // Build Jacobian (3x3)
    std::array<std::array<float, 3>, 3> J{};

    // Row 0: x component
    J[0][0] = 0.0f;
    J[0][1] = -leg_length_eff * std::cos(leg_angle_eff);
    J[0][2] = (calf_length_ * thigh_length_ * std::sin(calf_angle) * std::sin(leg_angle_eff) / leg_length_eff)
               - leg_length_eff * std::cos(leg_angle_eff) / 2.0f;

    // Row 1: y component
    J[1][0] = -hip_length_ * std::sin(hip_angle) +
        leg_length_eff * std::cos(hip_angle) * std::cos(leg_angle_eff);
    J[1][1] = -leg_length_eff * std::sin(hip_angle) * std::sin(leg_angle_eff);
    J[1][2] = (-calf_length_ * thigh_length_ * std::sin(hip_angle) * std::sin(calf_angle) *
               std::cos(leg_angle_eff) / leg_length_eff) -
        leg_length_eff * std::sin(hip_angle) * std::sin(leg_angle_eff) / 2.0f;

    // Row 2: z component
    J[2][0] = hip_length_ * std::cos(hip_angle) +
        leg_length_eff * std::sin(hip_angle) * std::cos(leg_angle_eff);
    J[2][1] = leg_length_eff * std::sin(leg_angle_eff) * std::cos(hip_angle);
    J[2][2] = (calf_length_ * thigh_length_ * std::sin(calf_angle) * std::cos(hip_angle) *
               std::cos(leg_angle_eff) / leg_length_eff) +
        leg_length_eff * std::sin(leg_angle_eff) * std::cos(hip_angle) / 2.0f;

    return J;
}

std::array<float, 3> LegKinematics::ComputeFootVelocity(
    const std::array<float, 3>& leg_angles,
    const std::array<float, 3>& joint_velocities,
    int leg_id) const
{
    auto J = AnalyticalLegJacobian(leg_angles, leg_id);

    // v_foot = J * dq
    std::array<float, 3> foot_velocity{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            foot_velocity[i] += J[i][j] * joint_velocities[j];
        }
    }

    // Base velocity is opposite: v_base = -v_foot (assuming stationary foot)
    for (int i = 0; i < 3; ++i)
    {
        foot_velocity[i] = -foot_velocity[i];
    }

    return foot_velocity;
}

// ============================================================================
// VelocityEstimator Implementation
// ============================================================================

std::array<float, 3> VelocityEstimator::MatrixVectorMultiply(
    const std::array<std::array<float, 3>, 3>& M,
    const std::array<float, 3>& v) const
{
    return {
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
    };
}

VelocityEstimator::VelocityEstimator(const Config& config)
    : config_(config)
    , kalman_filter_(
          config.accelerometer_variance,
          config.sensor_variance,
          config.initial_variance)
    , smoothing_filter_(config.moving_window_size, 3)
    , leg_kinematics_(config.hip_length, config.thigh_length, config.calf_length)
{
    estimated_velocity_.fill(0.0f);
}

void VelocityEstimator::Reset()
{
    kalman_filter_.Reset();
    smoothing_filter_.Reset();
    estimated_velocity_.fill(0.0f);
    last_timestamp_ns_ = 0;
    first_update_ = true;
}

void VelocityEstimator::Update(
    uint64_t timestamp_ns,
    const std::array<float, 3>& acceleration,
    const std::array<float, 3>& gyroscope,
    const std::array<float, 4>& quaternion,
    const std::array<float, 12>& joint_position,
    const std::array<float, 12>& joint_velocity,
    const std::array<float, 4>& foot_force)
{
    (void)gyroscope;

    // Compute delta time
    float dt = ComputeDeltaTime(timestamp_ns);

    // Get rotation matrix from quaternion
    auto rot_mat = QuaternionToRotationMatrix(quaternion);

    // Calibrate acceleration (remove bias)
    std::array<float, 3> calibrated_acc;
    for (int i = 0; i < 3; ++i)
    {
        calibrated_acc[i] = acceleration[i] - accelerometer_bias_[i];
    }

    // Transform acceleration to world frame and add gravity compensation
    std::array<float, 3> acc_world = MatrixVectorMultiply(rot_mat, calibrated_acc);
    for (int i = 0; i < 3; ++i)
    {
        acc_world[i] += config_.gravity[i];
    }

    // Predict step (IMU integration)
    kalman_filter_.Predict(dt, acc_world);

    // Determine foot contacts
    std::array<int, 4> foot_contact{};
    for (int i = 0; i < 4; ++i)
    {
        foot_contact[i] = (foot_force[i] > config_.foot_contact_force_threshold) ? 1 : 0;
    }

    // Correct using contact legs
    std::vector<std::array<float, 3>> observed_velocities;

    for (int leg_id = 0; leg_id < 4; ++leg_id)
    {
        if (foot_contact[leg_id])
        {
            // Get leg angles and velocities
            std::array<float, 3> leg_angles{
                joint_position[leg_id * 3 + 0],
                joint_position[leg_id * 3 + 1],
                joint_position[leg_id * 3 + 2]
            };
            std::array<float, 3> leg_velocities{
                joint_velocity[leg_id * 3 + 0],
                joint_velocity[leg_id * 3 + 1],
                joint_velocity[leg_id * 3 + 2]
            };

            // Compute base velocity from foot kinematics
            auto base_velocity_body = leg_kinematics_.ComputeFootVelocity(
                leg_angles, leg_velocities, leg_id);

            // Transform to world frame
            auto base_velocity_world = MatrixVectorMultiply(rot_mat, base_velocity_body);
            observed_velocities.push_back(base_velocity_world);
        }
    }

    // Update Kalman filter with average observation from contact legs
    if (!observed_velocities.empty())
    {
        std::array<float, 3> avg_observation{};
        for (const auto& obs : observed_velocities)
        {
            for (int i = 0; i < 3; ++i)
            {
                avg_observation[i] += obs[i];
            }
        }
        float scale = 1.0f / observed_velocities.size();
        for (int i = 0; i < 3; ++i)
        {
            avg_observation[i] *= scale;
        }

        kalman_filter_.Update(avg_observation);
    }

    // Kalman state is maintained in world frame. Convert back to body frame so the
    // public estimate matches IsaacLab's root_lin_vel_b convention used in dog-only.
    const auto kalman_output_world = kalman_filter_.GetState();
    const auto smoothed_velocity_world = smoothing_filter_.CalculateAverage(kalman_output_world);
    estimated_velocity_ = RotateInverse(quaternion, smoothed_velocity_world);
    for (int i = 0; i < 3; ++i)
    {
        estimated_velocity_[i] -= velocity_bias_[i];
    }
}

std::array<float, 3> VelocityEstimator::GetEstimatedVelocity() const
{
    return estimated_velocity_;
}

bool VelocityEstimator::IsReady() const
{
    return smoothing_filter_.IsReady();
}

void VelocityEstimator::SetAccelerometerBias(const std::array<float, 3>& bias)
{
    accelerometer_bias_ = bias;
}

void VelocityEstimator::SetVelocityBias(const std::array<float, 3>& bias)
{
    velocity_bias_ = bias;
}

float VelocityEstimator::ComputeDeltaTime(uint64_t timestamp_ns)
{
    if (first_update_)
    {
        last_timestamp_ns_ = timestamp_ns;
        first_update_ = false;
        return config_.default_control_dt;
    }

    uint64_t delta_ns = timestamp_ns - last_timestamp_ns_;
    last_timestamp_ns_ = timestamp_ns;

    // Convert to seconds, clamp to reasonable range
    float dt = static_cast<float>(delta_ns) * 1e-9f;
    return std::max(0.001f, std::min(0.1f, dt));  // Clamp between 1ms and 100ms
}

std::array<float, 3> VelocityEstimator::RotateInverse(
    const std::array<float, 4>& q,
    const std::array<float, 3>& v) const
{
    const auto rotation = QuaternionToRotationMatrix(q);
    return {
        rotation[0][0] * v[0] + rotation[1][0] * v[1] + rotation[2][0] * v[2],
        rotation[0][1] * v[0] + rotation[1][1] * v[1] + rotation[2][1] * v[2],
        rotation[0][2] * v[0] + rotation[1][2] * v[1] + rotation[2][2] * v[2]
    };
}

std::array<std::array<float, 3>, 3> VelocityEstimator::QuaternionToRotationMatrix(
    const std::array<float, 4>& q) const
{
    // q = [w, x, y, z]
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // Normalize
    float norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 1e-6f)
    {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }

    // Compute rotation matrix
    std::array<std::array<float, 3>, 3> R{};
    R[0][0] = 1.0f - 2.0f * (qy*qy + qz*qz);
    R[0][1] = 2.0f * (qx*qy - qw*qz);
    R[0][2] = 2.0f * (qx*qz + qw*qy);

    R[1][0] = 2.0f * (qx*qy + qw*qz);
    R[1][1] = 1.0f - 2.0f * (qx*qx + qz*qz);
    R[1][2] = 2.0f * (qy*qz - qw*qx);

    R[2][0] = 2.0f * (qx*qz - qw*qy);
    R[2][1] = 2.0f * (qy*qz + qw*qx);
    R[2][2] = 1.0f - 2.0f * (qx*qx + qy*qy);

    return R;
}

} // namespace rl_sar::state_estimation
