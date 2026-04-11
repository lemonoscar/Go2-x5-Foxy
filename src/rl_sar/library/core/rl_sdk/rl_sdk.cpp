#include <cerrno>
#include "rl_sdk.hpp"
#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"
#include "rl_sar/go2x5/control/go2_x5_operator_control.hpp"

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    const auto go2_x5_operator_config =
        Go2X5OperatorControl::BuildConfig(this->params, this->layered_go2_x5_config);
    const bool exclusive_go2_x5_control =
        Go2X5OperatorControl::IsExclusiveGo2X5Control(this->robot_name, go2_x5_operator_config);

    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();

    this->motiontime++;

    const bool in_rl = (this->fsm.current_state_ &&
                        this->fsm.current_state_->GetStateName().find("RLLocomotion") != std::string::npos);
    if (Go2X5OperatorControl::ShouldTriggerKey1(this->control, this->robot_name, in_rl))
    {
        if (exclusive_go2_x5_control)
        {
            Go2X5OperatorControl::ApplyFixedCommand(
                &this->control, go2_x5_operator_config, 0.0f, 0.0f, 0.0f);
            std::cout << std::endl << LOGGER::INFO
                      << "Key[1] pressed: RL policy mode ON (exclusive real deploy control)"
                      << " x=" << this->control.x
                      << " y=" << this->control.y
                      << " yaw=" << this->control.yaw << std::endl;
        }
        else
        {
            const auto key1_mode = Go2X5OperatorControl::ResolveKey1Mode(go2_x5_operator_config);
            if (key1_mode == Go2X5ControlLogic::Key1Mode::Navigation)
            {
                Go2X5OperatorControl::ApplyNavigationMode(&this->control);
                std::cout << std::endl << LOGGER::INFO
                          << "Key[1] pressed: navigation mode ON (/cmd_vel enabled)" << std::endl;
            }
            else
            {
                Go2X5OperatorControl::ApplyFixedCommand(
                    &this->control, go2_x5_operator_config, 0.6f, 0.0f, 0.0f);
                std::cout << std::endl << LOGGER::INFO << "Key[1] pressed: fixed cmd x=" << this->control.x
                          << " y=" << this->control.y << " yaw=" << this->control.yaw << std::endl;
            }
        }
        this->control.last_keyboard = Input::Keyboard::Num1;
    }

    if (!exclusive_go2_x5_control)
    {
        Go2X5OperatorControl::ApplyManualVelocityKey(&this->control, this->control.current_keyboard);
    }
    if (!exclusive_go2_x5_control && Go2X5OperatorControl::IsStopKey(this->control.current_keyboard))
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        // Allow fixed-command (Key[1]) to be re-triggered after stop.
        this->control.last_keyboard = Input::Keyboard::Space;
    }
    if (!exclusive_go2_x5_control && Go2X5OperatorControl::IsZeroKey(this->control.current_keyboard))
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        if (this->control.last_keyboard != Input::Keyboard::Num5)
        {
            std::cout << std::endl << LOGGER::INFO << "Key[5] pressed: cmd zero" << std::endl;
            this->control.last_keyboard = Input::Keyboard::Num5;
        }
    }
    const bool nav_keyboard_down =
        Go2X5OperatorControl::IsNavKeyboardDown(this->control, exclusive_go2_x5_control);
    const bool nav_gamepad_down =
        Go2X5OperatorControl::IsNavGamepadDown(this->control, exclusive_go2_x5_control);
    const bool nav_toggle_requested =
        Go2X5OperatorControl::ShouldToggleNavigation(this->control, exclusive_go2_x5_control);
    if (nav_toggle_requested)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: "
                  << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
    this->control.nav_keyboard_latched = nav_keyboard_down;
    this->control.nav_gamepad_latched = nav_gamepad_down;
    if (exclusive_go2_x5_control)
    {
        this->control.navigation_mode = false;
    }
}
std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.Get<std::vector<std::string>>("observations"))
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo, mujoco and real robot, the coordinate system for angular velocity is in the body coordinate system.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(this->obs.ang_vel * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                if (i >= 0 && i < static_cast<int>(dof_pos_rel.size()))
                {
                    dof_pos_rel[static_cast<size_t>(i)] = 0.0f;
                }
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            const int target_dim = this->params.Get<int>("actions_observation_dim", 0);
            if (target_dim > 0 && static_cast<int>(this->obs.actions.size()) < target_dim)
            {
                std::vector<float> padded = this->obs.actions;
                padded.resize(static_cast<size_t>(target_dim), 0.0f);
                obs_list.push_back(std::move(padded));
            }
            else if (target_dim > 0 && static_cast<int>(this->obs.actions.size()) > target_dim)
            {
                std::vector<float> trimmed = this->obs.actions;
                trimmed.resize(static_cast<size_t>(target_dim));
                obs_list.push_back(std::move(trimmed));
            }
            else
            {
                obs_list.push_back(this->obs.actions);
            }
        }
        else if (observation == "height_scan")
        {
            obs_list.push_back(this->obs.height_scan);
        }
        else if (observation == "arm_joint_command")
        {
            obs_list.push_back(this->obs.arm_joint_command);
        }
        else if (observation == "gripper_command")
        {
            obs_list.push_back({this->params.Get<float>("gripper_command_default", 0.0f)});
        }
        // ============= Other Observations =============
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    const int mapped = joint_mapping[i];
                    if (mapped >= 0 &&
                        mapped < static_cast<int>(joint_pos_sdk.size()) &&
                        mapped < static_cast<int>(joint_vel_sdk.size()))
                    {
                        joint_pos_training[i] = joint_pos_sdk[static_cast<size_t>(mapped)];
                        joint_vel_training[i] = joint_vel_sdk[static_cast<size_t>(mapped)];
                    }
                    else
                    {
                        joint_pos_training[i] = 0.0f;
                        joint_vel_training[i] = 0.0f;
                    }
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->motion_loader)
            {
                auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                if (waist_sdk_indices.size() >= 3)
                {
                    const int mapped0 = InverseJointMapping(waist_sdk_indices[0]);
                    const int mapped1 = InverseJointMapping(waist_sdk_indices[1]);
                    const int mapped2 = InverseJointMapping(waist_sdk_indices[2]);
                    const bool valid_mapping =
                        mapped0 >= 0 && mapped1 >= 0 && mapped2 >= 0 &&
                        mapped0 < static_cast<int>(this->obs.dof_pos.size()) &&
                        mapped1 < static_cast<int>(this->obs.dof_pos.size()) &&
                        mapped2 < static_cast<int>(this->obs.dof_pos.size());
                    if (valid_mapping)
                    {
                        std::vector<float> waist_angles = {
                            this->obs.dof_pos[static_cast<size_t>(mapped0)],
                            this->obs.dof_pos[static_cast<size_t>(mapped1)],
                            this->obs.dof_pos[static_cast<size_t>(mapped2)]
                        };
                        std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                        std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                        std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                        std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                        std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                        std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                        std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                        anchor_ori = MatrixFirstTwoColumns(rot_matrix);
                    }
                }
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = 0.0f;
            if (this->motion_length > 1e-6f)
            {
                phase = count / this->motion_length;
            }
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.height_scan.clear();
    this->obs.height_scan.resize(this->params.Get<int>("height_scan_size", 0), 0.0f);
    this->obs.arm_joint_command.clear();
    this->obs.arm_joint_command.resize(this->params.Get<int>("arm_command_size", 0), 0.0f);
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    this->ReadYaml(robot_config_path, "config.yaml");

    // init joint num first
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // init rl
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // init obs history
    const auto& observations_history = this->params.Get<std::vector<int>>("observations_history");  // avoid dangling reference
    if (!observations_history.empty())
    {
        int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.Get<std::string>("observations_history_priority"));
    }

    // init model
    std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!this->model)
    {
        throw std::runtime_error("Failed to load model from: " + model_path);
    }
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    // Action interface checklist:
    // 1) Policy outputs joint position offsets (same units as default_dof_pos; expected radians),
    //    except wheel_indices which are treated as velocity commands.
    // 2) Controller type is effort via PD (RobotJointController/Group uses q, dq, kp, kd, tau).
    // 3) No unit conversions or sign flips here; action_scale + default_dof_pos are applied directly.
    // 4) Action clipping happens in Forward() via clip_actions_*; no extra filtering/slew here.
    const auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    const auto action_scale = this->params.Get<std::vector<float>>("action_scale");
    const auto rl_kp = this->params.Get<std::vector<float>>("rl_kp");
    const auto rl_kd = this->params.Get<std::vector<float>>("rl_kd");
    const auto torque_limits = this->params.Get<std::vector<float>>("torque_limits");
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    const size_t full_dim = !default_dof_pos.empty()
        ? default_dof_pos.size()
        : static_cast<size_t>(std::max(0, num_dofs));

    output_dof_pos = default_dof_pos;
    if (output_dof_pos.size() < full_dim)
    {
        output_dof_pos.resize(full_dim, 0.0f);
    }
    output_dof_vel.assign(full_dim, 0.0f);
    output_dof_tau.assign(full_dim, 0.0f);

    std::vector<float> pos_actions_scaled(full_dim, 0.0f);
    std::vector<float> vel_actions_scaled(full_dim, 0.0f);
    const size_t action_dim = actions.size();
    for (size_t i = 0; i < action_dim && i < full_dim; ++i)
    {
        const float scale = (i < action_scale.size()) ? action_scale[i] : 1.0f;
        pos_actions_scaled[i] = actions[i] * scale;
    }
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        if (i >= 0 && static_cast<size_t>(i) < full_dim)
        {
            vel_actions_scaled[static_cast<size_t>(i)] = pos_actions_scaled[static_cast<size_t>(i)];
            pos_actions_scaled[static_cast<size_t>(i)] = 0.0f;
        }
    }

    std::vector<float> all_actions_scaled(full_dim, 0.0f);
    for (size_t i = 0; i < full_dim; ++i)
    {
        all_actions_scaled[i] = pos_actions_scaled[i] + vel_actions_scaled[i];
        output_dof_pos[i] += pos_actions_scaled[i];
        output_dof_vel[i] = vel_actions_scaled[i];
    }

    for (size_t i = 0; i < full_dim; ++i)
    {
        const float kp = (i < rl_kp.size()) ? rl_kp[i] : 0.0f;
        const float kd = (i < rl_kd.size()) ? rl_kd[i] : 0.0f;
        const float q_obs = (i < this->obs.dof_pos.size()) ? this->obs.dof_pos[i] : 0.0f;
        const float dq_obs = (i < this->obs.dof_vel.size()) ? this->obs.dof_vel[i] : 0.0f;
        output_dof_tau[i] = kp * (all_actions_scaled[i] + output_dof_pos[i] - pos_actions_scaled[i] - q_obs) - kd * dq_obs;
    }
    output_dof_tau = clamp(output_dof_tau, -torque_limits, torque_limits);
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[i];
        float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[index];
            float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

namespace
{
struct KeyboardInputState
{
    bool initialized = false;
    bool cleanup_registered = false;
    bool warned_no_tty = false;
    bool warned_read_error = false;
    int fd = -1;
    termios original_term{};
};

KeyboardInputState& GetKeyboardInputState()
{
    static KeyboardInputState state;
    return state;
}

void RestoreKeyboardInput()
{
    auto& state = GetKeyboardInputState();
    if (state.fd >= 0)
    {
        tcsetattr(state.fd, TCSANOW, &state.original_term);
        if (state.fd != STDIN_FILENO)
        {
            close(state.fd);
        }
    }

    state.fd = -1;
    state.initialized = false;
}

bool ConfigureKeyboardFd(const int fd, termios* original_term)
{
    if (fd < 0 || !original_term)
    {
        return false;
    }

    if (tcgetattr(fd, original_term) != 0)
    {
        return false;
    }

    termios new_term = *original_term;
    new_term.c_lflag &= ~(ICANON | ECHO);
    new_term.c_cc[VMIN] = 0;
    new_term.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &new_term) != 0)
    {
        return false;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1)
    {
        static_cast<void>(fcntl(fd, F_SETFL, flags | O_NONBLOCK));
    }

    return true;
}

int AcquireKeyboardFd()
{
    auto& state = GetKeyboardInputState();
    if (state.initialized)
    {
        return state.fd;
    }

    if (isatty(STDIN_FILENO) && ConfigureKeyboardFd(STDIN_FILENO, &state.original_term))
    {
        state.fd = STDIN_FILENO;
    }
    else
    {
        const int tty_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
        if (tty_fd >= 0 && isatty(tty_fd) && ConfigureKeyboardFd(tty_fd, &state.original_term))
        {
            state.fd = tty_fd;
            std::cout << LOGGER::INFO
                      << "Keyboard input attached to /dev/tty fallback for ros2 launch."
                      << std::endl;
        }
        else
        {
            if (tty_fd >= 0)
            {
                close(tty_fd);
            }
            if (!state.warned_no_tty)
            {
                std::cout << LOGGER::WARNING
                          << "Keyboard input unavailable: neither STDIN nor /dev/tty is interactive."
                          << std::endl;
                state.warned_no_tty = true;
            }
            state.initialized = true;
            state.fd = -1;
            return -1;
        }
    }

    if (!state.cleanup_registered)
    {
        std::atexit(RestoreKeyboardInput);
        state.cleanup_registered = true;
    }

    state.initialized = true;
    return state.fd;
}

int ReadKeyboardByte(const int fd)
{
    if (fd < 0)
    {
        return -1;
    }

    char c;
    const int result = read(fd, &c, 1);
    if (result == 1)
    {
        return static_cast<unsigned char>(c);
    }

    auto& state = GetKeyboardInputState();
    if (result < 0 &&
        errno != EAGAIN &&
        errno != EWOULDBLOCK &&
        errno != EINTR &&
        !state.warned_read_error)
    {
        std::cout << LOGGER::WARNING
                  << "Keyboard input read failed, errno=" << errno
                  << ". Controls may require a direct interactive TTY."
                  << std::endl;
        state.warned_read_error = true;
    }

    return -1;
}

} // namespace

void RL::KeyboardInterface()
{
    const int keyboard_fd = AcquireKeyboardFd();
    int c = ReadKeyboardByte(keyboard_fd);
    if (c > 0)
    {
        switch (c)
        {
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
        {
            // Try to read escape sequence non-blockingly
            const int seq0 = ReadKeyboardByte(keyboard_fd);
            if (seq0 == '[')
            {
                const int seq1 = ReadKeyboardByte(keyboard_fd);
                switch (seq1)
                {
                case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
                case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
                case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
                case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
                default: this->control.SetKeyboard(Input::Keyboard::Escape); break;
                }
            }
            else
            {
                // Plain escape key
                this->control.SetKeyboard(Input::Keyboard::Escape);
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    if (RLConfig::IsGo2X5ConfigPath(file_path))
    {
        this->layered_go2_x5_config.ApplyLoad(file_path, file_name, config_path, config);
        if (this->layered_go2_x5_config.HasValidationErrors())
        {
            throw std::runtime_error(this->layered_go2_x5_config.ValidationSummary());
        }
        this->params.config_node = this->layered_go2_x5_config.MergedNode();
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.config_node[key] = it->second;
    }
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(POLICY_DIR) + "/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    if (!rl.legacy_rl_output_path_enabled)
    {
        return;
    }

    RLCommandOutput output;
    bool has_output = false;
    while (rl.output_cmd_queue.try_pop(output))
    {
        has_output = true;
    }
    if (has_output)
    {
        const int num_dofs = rl.params.Get<int>("num_of_dofs");
        for (int i = 0; i < num_dofs; ++i)
        {
            if (static_cast<size_t>(i) < output.pos.size())
            {
                fsm_command->motor_command.q[i] = output.pos[static_cast<size_t>(i)];
            }
            if (static_cast<size_t>(i) < output.vel.size())
            {
                fsm_command->motor_command.dq[i] = output.vel[static_cast<size_t>(i)];
            }
            fsm_command->motor_command.kp[i] = rl.params.Get<std::vector<float>>("rl_kp")[i];
            fsm_command->motor_command.kd[i] = rl.params.Get<std::vector<float>>("rl_kd")[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    // Optional: lock arm joints to default pose (for go2_x5 stabilization / leg-only motion).
    if (rl.params.Get<bool>("arm_lock", false))
    {
        const int num_dofs = rl.params.Get<int>("num_of_dofs");
        const int arm_size = rl.params.Get<int>("arm_command_size", 0);
        if (arm_size > 0 && num_dofs >= arm_size)
        {
            const int arm_start = rl.params.Get<int>("arm_joint_start_index", std::max(0, num_dofs - arm_size));
            const auto default_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
            const auto arm_lock_pose = rl.arm_lock_pose_runtime_valid &&
                                               rl.arm_lock_pose_runtime.size() == static_cast<size_t>(arm_size)
                                           ? rl.arm_lock_pose_runtime
                                           : rl.params.Get<std::vector<float>>("arm_lock_pose");
            const auto fixed_kp = rl.params.Get<std::vector<float>>("fixed_kp");
            const auto fixed_kd = rl.params.Get<std::vector<float>>("fixed_kd");
            for (int i = 0; i < arm_size; ++i)
            {
                const int idx = arm_start + i;
                if (idx >= num_dofs || idx >= static_cast<int>(default_pos.size()))
                {
                    continue;
                }
                if (arm_lock_pose.size() == static_cast<size_t>(arm_size))
                {
                    fsm_command->motor_command.q[idx] = arm_lock_pose[static_cast<size_t>(i)];
                }
                else
                {
                    fsm_command->motor_command.q[idx] = default_pos[static_cast<size_t>(idx)];
                }
                fsm_command->motor_command.dq[idx] = 0.0f;
                if (idx < static_cast<int>(fixed_kp.size()))
                {
                    fsm_command->motor_command.kp[idx] = fixed_kp[static_cast<size_t>(idx)];
                }
                if (idx < static_cast<int>(fixed_kd.size()))
                {
                    fsm_command->motor_command.kd[idx] = fixed_kd[static_cast<size_t>(idx)];
                }
                fsm_command->motor_command.tau[idx] = 0.0f;
            }
        }
    }
}
