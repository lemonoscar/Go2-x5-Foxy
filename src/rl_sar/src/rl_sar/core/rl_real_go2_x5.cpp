
#include "rl_sar/core/rl_real_go2_x5.hpp"
#include "rl_sar/adapters/arx_adapter.hpp"
#include "rl_sar/adapters/unitree_adapter.hpp"
#include "rl_sar/diagnostics/diagnostics_publisher.hpp"
#include "rl_sar/diagnostics/drift_recorder.hpp"
#include "rl_sar/go2x5/arm/go2_x5_arm_controller.hpp"
#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"
#include "rl_sar/go2x5/state/go2_x5_state_manager.hpp"
#include "rl_sar/logger/event_logger.hpp"
#include "rl_sar/go2x5/safety/go2_x5_safety_guard.hpp"
#include "fsm_go2_x5.hpp"
#include "library/core/config/config_loader.hpp"
#include "library/core/config/deploy_manifest_runtime.hpp"
#include "matplotlibcpp.h"
#include "loop.hpp"
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>

#if defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace plt = matplotlibcpp;

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelSubscriber;

namespace
{

struct Go2X5RuntimeOptions
{
    bool enable_ros2_runtime = true;
    bool enable_ros2_runtime_explicit = false;
    std::string arm_bridge_transport = "ros";
    bool arm_bridge_transport_explicit = false;
    std::string manifest_path = RLConfig::DeployManifestRuntime::kDefaultManifestPath;
    std::string arm_bridge_ipc_host = Go2X5IPC::kDefaultHost;
    int arm_bridge_cmd_port = Go2X5IPC::kDefaultCommandPort;
    int arm_bridge_state_port = Go2X5IPC::kDefaultStatePort;
    int arm_joint_command_port = Go2X5IPC::kDefaultJointCommandPort;
};

std::string ToLowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

uint64_t TimePointAgeUs(const std::chrono::steady_clock::time_point& stamp,
                        const std::chrono::steady_clock::time_point& now)
{
    if (stamp.time_since_epoch().count() == 0 || now < stamp)
    {
        return std::numeric_limits<uint64_t>::max();
    }
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - stamp).count());
}

uint64_t TimePointToMonotonicNs(const std::chrono::steady_clock::time_point& stamp,
                                const uint64_t fallback_ns)
{
    if (stamp.time_since_epoch().count() == 0)
    {
        return fallback_ns;
    }
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            stamp.time_since_epoch()).count());
}

bool IsPassiveBodyOutputMode(const Go2X5Supervisor::Mode mode)
{
    switch (mode)
    {
    case Go2X5Supervisor::Mode::Boot:
    case Go2X5Supervisor::Mode::Probe:
    case Go2X5Supervisor::Mode::Passive:
    case Go2X5Supervisor::Mode::SoftStop:
    case Go2X5Supervisor::Mode::FaultLatched:
        return true;
    case Go2X5Supervisor::Mode::Ready:
    case Go2X5Supervisor::Mode::RlDogOnlyActive:
    case Go2X5Supervisor::Mode::ManualArm:
    case Go2X5Supervisor::Mode::DegradedArm:
    case Go2X5Supervisor::Mode::DegradedBody:
        return false;
    }
    return true;
}

bool IsPassiveBodyJointCommand(const float q,
                               const float dq,
                               const float kp,
                               const float kd,
                               const float tau)
{
    return q == static_cast<float>(PosStopF) &&
           dq == static_cast<float>(VelStopF) &&
           std::fabs(kp) <= 1e-6f &&
           std::fabs(kd) <= 1e-6f &&
           std::fabs(tau) <= 1e-6f;
}

bool ParseBoolArgValue(const std::string& raw_value, const bool fallback)
{
    const std::string value = ToLowerCopy(raw_value);
    if (value == "1" || value == "true" || value == "yes" || value == "on")
    {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off")
    {
        return false;
    }
    return fallback;
}

int ParseIntArgValue(const std::string& raw_value, const int fallback)
{
    try
    {
        return std::stoi(raw_value);
    }
    catch (...)
    {
        return fallback;
    }
}

std::filesystem::path ResolveProjectRootFromManifestPath(const std::string& manifest_path)
{
    if (!manifest_path.empty())
    {
        std::filesystem::path candidate(manifest_path);
        if (!candidate.is_absolute())
        {
            candidate = std::filesystem::absolute(candidate);
        }
        const auto manifest_dir = candidate.parent_path();
        if (manifest_dir.filename() == "deploy")
        {
            return manifest_dir.parent_path();
        }
    }

    const std::filesystem::path cwd = std::filesystem::current_path();
    if (std::filesystem::exists(cwd / "policy" / "go2_x5" / "base.yaml"))
    {
        return cwd;
    }
    return cwd;
}

std::string GetPolicyPathForRuntime(const std::string& manifest_path)
{
    std::filesystem::path policy_root = ResolveProjectRootFromManifestPath(manifest_path) / "policy";
    policy_root /= "";
    return policy_root.string();
}

Go2X5RuntimeOptions ParseGo2X5RuntimeOptions(int argc, char **argv)
{
    Go2X5RuntimeOptions options;
    for (int i = 2; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--disable-ros2-runtime")
        {
            options.enable_ros2_runtime = false;
            options.enable_ros2_runtime_explicit = true;
            continue;
        }
        if (arg == "--enable-ros2-runtime" && (i + 1) < argc)
        {
            options.enable_ros2_runtime = ParseBoolArgValue(argv[++i], options.enable_ros2_runtime);
            options.enable_ros2_runtime_explicit = true;
            continue;
        }
        if (arg == "--arm-bridge-transport" && (i + 1) < argc)
        {
            options.arm_bridge_transport = argv[++i];
            options.arm_bridge_transport_explicit = true;
            continue;
        }
        if (arg == "--manifest-path" && (i + 1) < argc)
        {
            options.manifest_path = argv[++i];
            continue;
        }
        if (arg == "--arm-bridge-ipc-host" && (i + 1) < argc)
        {
            options.arm_bridge_ipc_host = argv[++i];
            continue;
        }
        if (arg == "--arm-bridge-cmd-port" && (i + 1) < argc)
        {
            options.arm_bridge_cmd_port = ParseIntArgValue(argv[++i], options.arm_bridge_cmd_port);
            continue;
        }
        if (arg == "--arm-bridge-state-port" && (i + 1) < argc)
        {
            options.arm_bridge_state_port = ParseIntArgValue(argv[++i], options.arm_bridge_state_port);
            continue;
        }
        if (arg == "--arm-joint-cmd-port" && (i + 1) < argc)
        {
            options.arm_joint_command_port = ParseIntArgValue(argv[++i], options.arm_joint_command_port);
            continue;
        }
    }

    if (!options.arm_bridge_transport_explicit && !options.enable_ros2_runtime)
    {
        options.arm_bridge_transport = "ipc";
    }
    options.arm_bridge_transport = Go2X5IPC::NormalizeTransport(options.arm_bridge_transport);
    if (options.arm_bridge_transport != "ros" && !Go2X5IPC::IsIpcTransport(options.arm_bridge_transport))
    {
        options.arm_bridge_transport = options.enable_ros2_runtime ? "ros" : "ipc";
    }
    if (options.arm_bridge_ipc_host.empty())
    {
        options.arm_bridge_ipc_host = Go2X5IPC::kDefaultHost;
    }
    if (options.arm_bridge_cmd_port <= 0)
    {
        options.arm_bridge_cmd_port = Go2X5IPC::kDefaultCommandPort;
    }
    if (options.arm_bridge_state_port <= 0)
    {
        options.arm_bridge_state_port = Go2X5IPC::kDefaultStatePort;
    }
    if (options.arm_joint_command_port <= 0)
    {
        options.arm_joint_command_port = Go2X5IPC::kDefaultJointCommandPort;
    }
    options.manifest_path = RLConfig::DeployManifestRuntime::NormalizeManifestPath(options.manifest_path);
    return options;
}

#if defined(USE_ROS2) && defined(USE_ROS)
std::vector<char*> BuildRos2Argv(int argc, char **argv)
{
    std::vector<char*> filtered;
    filtered.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const bool consumes_next =
            (arg == "--enable-ros2-runtime" ||
             arg == "--manifest-path" ||
             arg == "--arm-bridge-transport" ||
             arg == "--arm-bridge-ipc-host" ||
             arg == "--arm-bridge-cmd-port" ||
             arg == "--arm-bridge-state-port" ||
             arg == "--arm-joint-cmd-port");
        if (arg == "--disable-ros2-runtime")
        {
            continue;
        }
        if (consumes_next)
        {
            ++i;
            continue;
        }
        filtered.push_back(argv[i]);
    }
    return filtered;
}
#endif

#if defined(__linux__)
bool MakeIpv4Endpoint(const std::string& host, const int port, sockaddr_in& addr)
{
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    return inet_pton(AF_INET, host.c_str(), &addr.sin_addr) == 1;
}
#endif

rl_sar::logger::ModeType ToLoggerMode(const Go2X5Supervisor::Mode mode)
{
    using rl_sar::logger::ModeType;
    switch (mode)
    {
    case Go2X5Supervisor::Mode::Boot: return ModeType::BOOT;
    case Go2X5Supervisor::Mode::Probe: return ModeType::PROBE;
    case Go2X5Supervisor::Mode::Passive: return ModeType::PASSIVE;
    case Go2X5Supervisor::Mode::Ready: return ModeType::READY;
    case Go2X5Supervisor::Mode::RlDogOnlyActive: return ModeType::RL_DOG_ONLY_ACTIVE;
    case Go2X5Supervisor::Mode::ManualArm: return ModeType::MANUAL_ARM;
    case Go2X5Supervisor::Mode::DegradedArm: return ModeType::DEGRADED_ARM;
    case Go2X5Supervisor::Mode::DegradedBody: return ModeType::DEGRADED_BODY;
    case Go2X5Supervisor::Mode::SoftStop: return ModeType::SOFT_STOP;
    case Go2X5Supervisor::Mode::FaultLatched: return ModeType::FAULT_LATCHED;
    }
    return ModeType::BOOT;
}

rl_sar::logger::FaultType ToLoggerFault(const Go2X5Supervisor::ReasonCode code)
{
    using rl_sar::logger::FaultType;
    switch (code)
    {
    case Go2X5Supervisor::ReasonCode::BodyStateStale: return FaultType::BODY_STATE_STALE;
    case Go2X5Supervisor::ReasonCode::ArmStateStale: return FaultType::ARM_STATE_STALE;
    case Go2X5Supervisor::ReasonCode::PolicyTimeout: return FaultType::POLICY_STALE;
    case Go2X5Supervisor::ReasonCode::ArmTrackingErrorTooHigh: return FaultType::ARM_TRACKING_ERROR;
    case Go2X5Supervisor::ReasonCode::DdsWriteFail: return FaultType::DDS_FAILURE;
    default: return FaultType::UNKNOWN;
    }
}

} // namespace

RL_Real_Go2X5::RL_Real_Go2X5(int argc, char **argv)
{
    std::cout << LOGGER::INFO << "[Boot] go2_x5 constructor begin" << std::endl;

    this->InitializeRuntimeOptions(argc, argv);
    this->InitializeConfigLoader();

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "go2_x5";
    this->ReadYaml(this->robot_name, "base.yaml");
    this->InitializeArmConfig();
    this->legacy_rl_output_path_enabled = false;
    this->ValidateJointMappingOrThrow("base.yaml");
    std::cout << LOGGER::INFO << "[Boot] base.yaml loaded and arm config validated" << std::endl;

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }
    std::cout << LOGGER::INFO << "[Boot] FSM initialization complete" << std::endl;

#if defined(USE_ROS1) && defined(USE_ROS)
    std::cout << LOGGER::INFO << "[Boot] Creating ROS1 node handle" << std::endl;
    this->ros1_nh = std::make_shared<ros::NodeHandle>();
    std::cout << LOGGER::INFO << "[Boot] Creating /cmd_vel publisher" << std::endl;
    this->cmd_vel_publisher = this->ros1_nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->cmd_vel_subscriber = this->ros1_nh->subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real_Go2X5::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    if (this->enable_ros2_runtime)
    {
        std::cout << LOGGER::INFO << "[Boot] Creating ROS2 node" << std::endl;
        ros2_node = std::make_shared<rclcpp::Node>("rl_real_go2_x5_node");
        std::cout << LOGGER::INFO << "[Boot] Creating /cmd_vel publisher" << std::endl;
        this->cmd_vel_publisher = ros2_node->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS());
        std::cout << LOGGER::INFO << "[Boot] Creating /cmd_vel subscription" << std::endl;
        this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this] (const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdvelCallback(msg); });
    }
    else
    {
        std::cout << LOGGER::INFO << "[Boot] ROS2 runtime disabled for rl_real_go2_x5" << std::endl;
    }
#endif
    std::cout << LOGGER::INFO << "[Boot] Runtime command channel setup complete" << std::endl;

    std::cout << LOGGER::INFO << "[Boot] Setting up arm command subscriber" << std::endl;
    this->SetupArmCommandSubscriber();
    std::cout << LOGGER::INFO << "[Boot] Setting up arm bridge interface" << std::endl;
    this->SetupArmBridgeInterface();
    std::cout << LOGGER::INFO << "[Boot] Arm ROS interfaces ready" << std::endl;

    // init robot
    std::cout << LOGGER::INFO << "[Boot] Initializing low-level command/state buffers" << std::endl;
    this->InitLowCmd();
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    std::cout << LOGGER::INFO << "[Boot] Low-level buffers ready" << std::endl;

    // create lowcmd publisher
    std::cout << LOGGER::INFO << "[Boot] Creating Unitree lowcmd publisher" << std::endl;
    this->lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    this->lowcmd_publisher->InitChannel();
    // create lowstate subscriber
    std::cout << LOGGER::INFO << "[Boot] Creating Unitree lowstate subscriber" << std::endl;
    this->lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    this->lowstate_subscriber->InitChannel(std::bind(&RL_Real_Go2X5::LowStateMessageHandler, this, std::placeholders::_1), 1);
    // create joystick subscriber
    std::cout << LOGGER::INFO << "[Boot] Creating Unitree joystick subscriber" << std::endl;
    this->joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    this->joystick_subscriber->InitChannel(std::bind(&RL_Real_Go2X5::JoystickHandler, this, std::placeholders::_1), 1);
    std::cout << LOGGER::INFO << "[Boot] Unitree DDS channels ready" << std::endl;

    // init MotionSwitcherClient
    std::cout << LOGGER::INFO << "[Boot] Initializing MotionSwitcherClient" << std::endl;
    this->msc.SetTimeout(10.0f);
    this->msc.Init();
    std::cout << LOGGER::INFO << "[Boot] MotionSwitcherClient ready" << std::endl;
    // Shut down motion control-related service
    while (this->QueryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = this->msc.ReleaseMode();
        if (ret == 0)
        {
            std::cout << "ReleaseMode succeeded." << std::endl;
        }
        else
        {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(1);
    }
    std::cout << LOGGER::INFO << "[Boot] Motion control-related service released" << std::endl;

    this->InitializeRuntimeIntegrations();
    this->RefreshSupervisorState("boot");

    std::cout << LOGGER::INFO << "Real deploy target: go2_x5" << std::endl;
    std::cout << LOGGER::INFO << "arm_joint_command_topic: " << this->arm_joint_command_topic
              << ", arm_hold_enabled: " << (this->arm_hold_enabled ? "true" : "false")
              << ", arm_lock: " << (this->params.Get<bool>("arm_lock", false) ? "true" : "false")
              << std::endl;

    // loop
    std::cout << LOGGER::INFO << "[Boot] Starting control loops" << std::endl;
    auto loop_exception_handler = [this](const std::string& loop_name, const std::string& error)
    {
        this->HandleLoopException(loop_name, error);
    };
    const float dt = GetDt();
    const int decimation = GetDecimation();
    this->loop_keyboard = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RL_Real_Go2X5::KeyboardInterface, this), -1, loop_exception_handler);
    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control", dt, std::bind(&RL_Real_Go2X5::RobotControl, this), -1, loop_exception_handler);
    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl",
        dt * decimation,
        std::bind(&RL_Real_Go2X5::RunModel, this),
        -1,
        loop_exception_handler);
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();
    std::cout << LOGGER::INFO << "[Boot] Control loops started" << std::endl;

#ifdef PLOT
    const int num_dofs = GetNumDofs();
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(num_dofs);
    this->plot_target_joint_pos.resize(num_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>(
        "loop_plot", 0.002, std::bind(&RL_Real_Go2X5::Plot, this), -1, loop_exception_handler);
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real_Go2X5::~RL_Real_Go2X5()
{
    this->SafeShutdownNow();
    if (this->diagnostics_publisher_)
    {
        this->diagnostics_publisher_->Stop();
    }
    if (this->drift_recorder_)
    {
        this->drift_recorder_->StopRecordingWindow();
    }
    if (this->event_logger_)
    {
        this->event_logger_->Shutdown();
    }
    if (this->arx_adapter_)
    {
        this->arx_adapter_->Stop();
    }
    if (this->unitree_adapter_)
    {
        this->unitree_adapter_->Stop();
    }
    // Restore built-in motion service so wireless controller can take over after process exit.
    if (!this->QueryMotionStatus())
    {
        int32_t ret = this->msc.SelectMode("normal");
        if (ret != 0)
        {
            ret = this->msc.SelectMode("sport_mode");
        }
        if (ret == 0)
        {
            std::cout << LOGGER::INFO << "Restored motion service: normal(sport_mode)" << std::endl;
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "Failed to restore motion service with alias 'normal', error: " << ret << std::endl;
        }
    }
    std::cout << LOGGER::INFO << "RL_Real_Go2X5 exit" << std::endl;
}

// Safe shutdown functions moved to go2_x5_safe_shutdown.cpp

void RL_Real_Go2X5::InitializeRuntimeOptions(int argc, char **argv)
{
    const auto options = ParseGo2X5RuntimeOptions(argc, argv);
    this->enable_ros2_runtime = options.enable_ros2_runtime;
    this->runtime_ros2_enabled_explicit_ = options.enable_ros2_runtime_explicit;
    this->deploy_manifest_path_ = options.manifest_path;
    this->arm_bridge_transport = options.arm_bridge_transport;
    this->runtime_arm_bridge_transport_explicit_ = options.arm_bridge_transport_explicit;
    this->arm_bridge_ipc_host = options.arm_bridge_ipc_host;
    this->arm_bridge_cmd_port = options.arm_bridge_cmd_port;
    this->arm_bridge_state_port = options.arm_bridge_state_port;
    this->arm_joint_command_port = options.arm_joint_command_port;
    std::cout << LOGGER::INFO
              << "[Boot] Runtime options: manifest_path=" << this->deploy_manifest_path_
              << ", ros2_runtime=" << (this->enable_ros2_runtime ? "true" : "false")
              << ", arm_bridge_transport=" << this->arm_bridge_transport
              << ", ipc_host=" << this->arm_bridge_ipc_host
              << ", cmd_port=" << this->arm_bridge_cmd_port
              << ", state_port=" << this->arm_bridge_state_port
              << ", joint_cmd_port=" << this->arm_joint_command_port
              << std::endl;
}

void RL_Real_Go2X5::InitializeConfigLoader()
{
    std::cout << LOGGER::INFO << "[Boot] Initializing ConfigLoader" << std::endl;
    config_loader_ = std::make_unique<RLConfig::ConfigLoader>();

    // Load base.yaml configuration
    const std::string policy_path = GetPolicyPathForRuntime(this->deploy_manifest_path_);
    const std::string base_yaml_path = policy_path + "go2_x5/base.yaml";
    auto result = config_loader_->LoadLayerFromScopedFile(
        RLConfig::ConfigLayer::BaseYaml, base_yaml_path, "go2_x5");
    if (!result.is_valid)
    {
        std::cout << LOGGER::WARNING << "Failed to load base.yaml: " << result.error_message << std::endl;
    }

    // Load robot_lab config if exists
    const std::string lab_yaml_path = policy_path + "go2_x5/robot_lab/config.yaml";
    struct stat buffer;
    if (stat(lab_yaml_path.c_str(), &buffer) == 0)
    {
        result = config_loader_->LoadLayerFromScopedFile(
            RLConfig::ConfigLayer::RuntimeYaml, lab_yaml_path, "go2_x5/robot_lab");
        if (!result.is_valid)
        {
            std::cout << LOGGER::WARNING << "Failed to load robot_lab/config.yaml: " << result.error_message << std::endl;
        }
    }

    deploy_manifest_runtime_ = std::make_unique<RLConfig::DeployManifestRuntime>(this->deploy_manifest_path_);
    const auto manifest_result = deploy_manifest_runtime_->LoadFromFile(this->deploy_manifest_path_);
    if (manifest_result.is_valid)
    {
        const auto& manifest = deploy_manifest_runtime_->Manifest();
        const auto snapshot = deploy_manifest_runtime_->Snapshot();
        manifest_hash_ = snapshot.manifest_hash;

        if (!this->runtime_ros2_enabled_explicit_)
        {
            this->enable_ros2_runtime = snapshot.ros2_enabled;
        }
        if (!this->runtime_arm_bridge_transport_explicit_)
        {
            this->arm_bridge_transport = snapshot.arm_bridge_transport;
        }

        YAML::Node dt_node;
        dt_node = static_cast<float>(1.0 / std::max(1, snapshot.coordinator_rate_hz));
        config_loader_->Set("dt", dt_node);

        if (snapshot.policy_rate_hz > 0 && snapshot.coordinator_rate_hz >= snapshot.policy_rate_hz &&
            (snapshot.coordinator_rate_hz % snapshot.policy_rate_hz) == 0)
        {
            YAML::Node decimation_node;
            decimation_node = snapshot.coordinator_rate_hz / snapshot.policy_rate_hz;
            config_loader_->Set("decimation", decimation_node);
        }

        YAML::Node arm_joint_count_node;
        arm_joint_count_node = manifest.robot.arm_joint_count;
        config_loader_->Set("arm_joint_count", arm_joint_count_node);

        YAML::Node arm_command_size_node;
        arm_command_size_node = manifest.robot.arm_joint_count;
        config_loader_->Set("arm_command_size", arm_command_size_node);

        YAML::Node arm_joint_start_index_node;
        arm_joint_start_index_node = manifest.robot.leg_joint_count;
        config_loader_->Set("arm_joint_start_index", arm_joint_start_index_node);

        YAML::Node arm_control_mode_node;
        arm_control_mode_node = std::string("split");
        config_loader_->Set("arm_control_mode", arm_control_mode_node);

        YAML::Node arm_bridge_require_state_node;
        arm_bridge_require_state_node = true;
        config_loader_->Set("arm_bridge_require_state", arm_bridge_require_state_node);

        YAML::Node arm_bridge_require_live_state_node;
        arm_bridge_require_live_state_node = manifest.arm_adapter.require_live_state;
        config_loader_->Set("arm_bridge_require_live_state", arm_bridge_require_live_state_node);

        YAML::Node arm_bridge_state_timeout_sec_node;
        arm_bridge_state_timeout_sec_node =
            static_cast<float>(std::max(0, manifest.arm_adapter.arm_state_timeout_ms) / 1000.0f);
        config_loader_->Set("arm_bridge_state_timeout_sec", arm_bridge_state_timeout_sec_node);
    }

    const auto schema_result = config_loader_->Validate(RLConfig::ConfigLoader::CreateGo2X5Schema());
    runtime_config_valid_ = schema_result.is_valid;
    if (!runtime_config_valid_)
    {
        std::cout << LOGGER::WARNING << "[Boot] go2_x5 layered config validation failed: "
                  << schema_result.error_message << " @ " << schema_result.error_path << std::endl;
        throw std::runtime_error(
            "go2_x5 runtime config invalid: " + schema_result.error_message +
            (schema_result.error_path.empty() ? std::string() : " @ " + schema_result.error_path));
    }

    manifest_valid_ = manifest_result.is_valid && runtime_config_valid_;
    if (!manifest_result.is_valid)
    {
        std::cout << LOGGER::WARNING << "[Boot] Deploy manifest load failed: "
                  << manifest_result.error_message << " @ " << manifest_result.error_path << std::endl;
    }
    else
    {
        const auto snapshot = deploy_manifest_runtime_->Snapshot();
        std::cout << LOGGER::INFO << "[Boot] Deploy manifest loaded: path="
                  << deploy_manifest_runtime_->ManifestPath()
                  << ", hash=" << manifest_hash_
                  << ", protocol_version=" << snapshot.protocol_version
                  << ", policy_id_hash=" << snapshot.policy_id_hash
                  << std::endl;
        std::cout << LOGGER::INFO
                  << "[Boot] Manifest effective runtime: ros2_enabled="
                  << (this->enable_ros2_runtime ? "true" : "false")
                  << ", transport=" << this->arm_bridge_transport
                  << ", manifest_ros2_enabled=" << (snapshot.ros2_enabled ? "true" : "false")
                  << ", manifest_transport=" << snapshot.arm_bridge_transport
                  << ", policy_rate_hz=" << snapshot.policy_rate_hz
                  << ", coordinator_rate_hz=" << snapshot.coordinator_rate_hz
                  << ", lowstate_timeout_ms=" << snapshot.lowstate_timeout_ms
                  << ", arm_state_timeout_ms=" << snapshot.arm_state_timeout_ms
                  << ", policy_timeout_ms=" << snapshot.policy_timeout_ms
                  << ", arm_background_send_recv="
                  << (snapshot.arm_background_send_recv ? "true" : "false")
                  << ", arm_controller_dt=" << snapshot.arm_controller_dt
                  << ", arm_cmd_topic=" << snapshot.arm_cmd_topic
                  << ", arm_state_topic=" << snapshot.arm_state_topic
                  << ", arm_joint_command_topic=" << snapshot.arm_joint_command_topic
                  << ", bridge_rmw=" << snapshot.bridge_rmw_implementation
                  << ", runtime_rmw=" << snapshot.go2_rmw_implementation
                  << std::endl;
    }

    // Create Go2X5Config wrapper
    config_ = std::make_unique<Go2X5Config::Go2X5Config>(*config_loader_);

    // Create StateManager
    state_manager_ = std::make_unique<Go2X5State::StateManager>(
        config_->GetNumDofs(),
        config_->GetArmJointCount()
    );

    InitializeSupervisor();

    std::cout << LOGGER::INFO << "[Boot] ConfigLoader initialized" << std::endl;
}

void RL_Real_Go2X5::InitializeArmConfig()
{
    std::cout << LOGGER::INFO << "[Boot] Initializing arm configuration" << std::endl;
    this->InitializeArmCommandState();
    this->InitializeArmChannelConfig();
    this->InitializeRealDeploySafetyConfig();
    this->InitializeCoordinator();
    std::cout << LOGGER::INFO << "[Boot] Arm configuration initialized" << std::endl;
}

void RL_Real_Go2X5::InitializeSupervisor()
{
    Go2X5Supervisor::Config supervisor_config;
    if (deploy_manifest_runtime_ && deploy_manifest_runtime_->HasManifest())
    {
        const auto& manifest = deploy_manifest_runtime_->Manifest();
        supervisor_config.probe_window_us =
            static_cast<uint64_t>(std::max(0.0, manifest.supervisor.probe_window_sec) * 1'000'000.0);
        supervisor_config.body_state_stale_us =
            static_cast<uint64_t>(std::max(0, manifest.body_adapter.lowstate_timeout_ms) * 1000ULL);
        supervisor_config.arm_state_stale_us =
            static_cast<uint64_t>(std::max(0, manifest.arm_adapter.arm_state_timeout_ms) * 1000ULL);
        supervisor_config.policy_stale_us =
            static_cast<uint64_t>(std::max(0, manifest.supervisor.policy_timeout_ms) * 1000ULL);
        supervisor_config.arm_tracking_error_window_us = 200'000ULL;
        this->arm_tracking_error_limit_ = manifest.arm_adapter.arm_tracking_error_limit;
        supervisor_config.degraded_timeout_us =
            static_cast<uint64_t>(std::max(0.0, manifest.supervisor.degraded_timeout_sec) * 1'000'000.0);
        supervisor_config.soft_stop_duration_us = 500'000ULL;
        supervisor_config.require_manifest_valid = true;
        supervisor_config.fault_latched_requires_manual_reset =
            manifest.supervisor.fault_latched_requires_manual_reset;
    }

    supervisor_ = std::make_unique<Go2X5Supervisor::Supervisor>(supervisor_config);
    std::cout << LOGGER::INFO << "[Boot] Supervisor initialized"
              << " probe_window_us=" << supervisor_->config().probe_window_us
              << " body_stale_us=" << supervisor_->config().body_state_stale_us
              << " arm_stale_us=" << supervisor_->config().arm_state_stale_us
              << " policy_stale_us=" << supervisor_->config().policy_stale_us
              << " manifest_valid=" << (manifest_valid_ ? "true" : "false")
              << std::endl;
}

void RL_Real_Go2X5::InitializeCoordinator()
{
    rl_sar::runtime::coordinator::Config coordinator_config;
    if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
    {
        const auto snapshot = this->deploy_manifest_runtime_->Snapshot();
        if (snapshot.body_command_expire_ms > 0)
        {
            coordinator_config.body_command_expire_ns =
                static_cast<uint64_t>(snapshot.body_command_expire_ms) * 1'000'000ULL;
        }
        if (snapshot.arm_command_expire_ms > 0)
        {
            coordinator_config.arm_command_expire_ns =
                static_cast<uint64_t>(snapshot.arm_command_expire_ms) * 1'000'000ULL;
        }
    }

    const auto default_dof_pos = this->GetDefaultDofPos();
    const auto fixed_kp = this->GetFixedKp();
    const auto fixed_kd = this->GetFixedKd();
    const auto action_scale = this->config_->GetActionScale();
    const auto torque_limits = this->GetDefaultWholeBodyEffortLimits();
    const int num_dofs = this->GetNumDofs();

    size_t body_idx = 0;
    for (int joint_idx = 0;
         joint_idx < num_dofs && body_idx < rl_sar::protocol::kBodyJointCount;
         ++joint_idx)
    {
        if (this->IsArmJointIndex(joint_idx))
        {
            continue;
        }

        const size_t body_slot = body_idx++;
        const size_t joint_slot = static_cast<size_t>(joint_idx);
        coordinator_config.default_leg_q[body_slot] =
            joint_slot < default_dof_pos.size() ? default_dof_pos[joint_slot] : 0.0f;
        coordinator_config.safe_stand_q[body_slot] =
            coordinator_config.default_leg_q[body_slot];
        coordinator_config.rl_kp[body_slot] =
            joint_slot < fixed_kp.size() ? fixed_kp[joint_slot] : 0.0f;
        coordinator_config.rl_kd[body_slot] =
            joint_slot < fixed_kd.size() ? fixed_kd[joint_slot] : 0.0f;

        float scale_value = 0.0f;
        if (joint_slot < action_scale.size())
        {
            scale_value = action_scale[joint_slot];
        }
        else if (body_slot < action_scale.size())
        {
            scale_value = action_scale[body_slot];
        }
        coordinator_config.action_scale[body_slot] = scale_value;

        float torque_limit = 0.0f;
        if (joint_slot < torque_limits.size() && std::isfinite(torque_limits[joint_slot]))
        {
            torque_limit = std::max(0.0f, std::fabs(torque_limits[joint_slot]));
        }
        coordinator_config.torque_limits[body_slot] = torque_limit;
    }

    this->coordinator_ =
        std::make_unique<rl_sar::runtime::coordinator::HybridMotionCoordinator>(coordinator_config);
    std::cout << LOGGER::INFO
              << "[Boot] Coordinator initialized"
              << " body_expire_ns=" << coordinator_config.body_command_expire_ns
              << " arm_expire_ns=" << coordinator_config.arm_command_expire_ns
              << std::endl;
}

void RL_Real_Go2X5::InitializeRuntimeIntegrations()
{
    this->InitializeAdapters();
    this->InitializeDiagnostics();
}

void RL_Real_Go2X5::InitializeAdapters()
{
    if (!this->deploy_manifest_runtime_ || !this->deploy_manifest_runtime_->HasManifest())
    {
        return;
    }

    const auto& manifest = this->deploy_manifest_runtime_->Manifest();

    this->unitree_adapter_ = std::make_unique<rl_sar::adapters::UnitreeAdapter>();
    rl_sar::adapters::UnitreeAdapter::Config body_config;
    body_config.network_interface = manifest.body_adapter.network_interface;
    body_config.command_rate_hz = manifest.body_adapter.command_rate_hz;
    body_config.require_lowstate = manifest.body_adapter.require_lowstate;
    body_config.lowstate_timeout_ms = static_cast<double>(manifest.body_adapter.lowstate_timeout_ms);
    body_config.leg_dof_count = static_cast<uint16_t>(manifest.robot.leg_joint_count);
    body_config.source_id = 1U;
    body_config.initialize_channel_factory = false;
    for (size_t i = 0; i < body_config.joint_mapping.size(); ++i)
    {
        body_config.joint_mapping[i] =
            (i < manifest.robot.joint_mapping.size())
                ? manifest.robot.joint_mapping[i]
                : static_cast<int>(i);
    }
    const auto body_init = this->unitree_adapter_->Initialize(body_config);
    if (body_init == rl_sar::adapters::UnitreeAdapter::Status::kOk &&
        this->unitree_adapter_->Start() == rl_sar::adapters::UnitreeAdapter::Status::kOk)
    {
        this->unitree_adapter_active_ = true;
        std::cout << LOGGER::INFO
                  << "[Boot] UnitreeAdapter active: iface=" << body_config.network_interface
                  << ", rate_hz=" << body_config.command_rate_hz
                  << std::endl;
    }
    else
    {
        this->unitree_adapter_active_ = false;
        std::cout << LOGGER::WARNING
                  << "[Boot] UnitreeAdapter init failed. Falling back to legacy Unitree path."
                  << std::endl;
    }

    const bool external_arm_pipeline_active =
        Go2X5IPC::IsIpcTransport(this->arm_bridge_transport) || this->arm_bridge_transport == "ros";
    if (this->arm_split_control_enabled && this->arm_joint_count > 0 && !external_arm_pipeline_active)
    {
        this->arx_adapter_ = std::make_unique<rl_sar::adapters::ArxAdapter>();
        rl_sar::adapters::ArxAdapter::Config arm_config;
        arm_config.can_interface = manifest.arm_adapter.can_interface;
        arm_config.target_rate_hz = manifest.arm_adapter.arm_target_rate_hz;
        arm_config.servo_rate_hz = manifest.arm_adapter.servo_rate_hz;
        arm_config.background_send_recv = manifest.arm_adapter.background_send_recv;
        arm_config.controller_dt = manifest.arm_adapter.controller_dt;
        arm_config.require_live_state = manifest.arm_adapter.require_live_state;
        arm_config.arm_state_timeout_ms = static_cast<double>(manifest.arm_adapter.arm_state_timeout_ms);
        arm_config.arm_tracking_error_limit = manifest.arm_adapter.arm_tracking_error_limit;
        if (const char* sdk_root = std::getenv("ARX5_SDK_ROOT"))
        {
            arm_config.sdk_root = sdk_root;
        }
        if (const char* sdk_lib_path = std::getenv("ARX5_SDK_LIB_PATH"))
        {
            arm_config.sdk_lib_path = sdk_lib_path;
        }
        this->arx_adapter_active_ = this->arx_adapter_->Initialize(arm_config);
        if (this->arx_adapter_active_)
        {
            this->arx_adapter_->Start();
            std::cout << LOGGER::INFO
                      << "[Boot] ArxAdapter active: can=" << arm_config.can_interface
                      << ", servo_rate_hz=" << arm_config.servo_rate_hz
                      << std::endl;
        }
        else
        {
            std::cout << LOGGER::WARNING
                      << "[Boot] ArxAdapter init failed. Falling back to bridge transport path."
                      << std::endl;
        }
    }
    else if (this->arm_split_control_enabled && this->arm_joint_count > 0)
    {
        this->arx_adapter_active_ = false;
        std::cout << LOGGER::INFO
                  << "[Boot] External arm pipeline owns hardware transport; ArxAdapter remains inactive."
                  << std::endl;
    }
}

void RL_Real_Go2X5::InitializeDiagnostics()
{
    this->diagnostics_publisher_ = std::make_unique<rl_sar::diagnostics::DiagnosticsPublisher>();
    this->drift_recorder_ = std::make_unique<rl_sar::diagnostics::DriftMetricsRecorder>();
    this->event_logger_ = std::make_unique<rl_sar::logger::EventLogger>();

    rl_sar::diagnostics::DiagnosticsPublisher::Config diagnostics_config;
    diagnostics_config.ros2_enabled = false;
    diagnostics_config.publish_rate_hz = 50;
    diagnostics_config.node_name = "go2_x5_diagnostics";
    diagnostics_config.diagnostics_topic = "/go2_x5/diagnostics";
    if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
    {
        const auto snapshot = this->deploy_manifest_runtime_->Snapshot();
        diagnostics_config.ros2_enabled = this->enable_ros2_runtime && snapshot.ros2_enabled;
        diagnostics_config.ros2_mirror_only = snapshot.ros2_mirror_only;
        diagnostics_config.publish_rate_hz = std::max(1, snapshot.diagnostics_rate_hz);
    }
    if (this->diagnostics_publisher_->Initialize(diagnostics_config))
    {
        this->diagnostics_publisher_->Start();
    }

    rl_sar::diagnostics::DriftMetricsRecorder::Config drift_config;
    drift_config.auto_start_on_zero_cmd = false;
    drift_config.output_csv_path = "/tmp/go2_x5_drift.csv";
    this->drift_recorder_->Initialize(drift_config);
    this->drift_recorder_->StartRecordingWindow();

    rl_sar::logger::EventLoggerConfig logger_config;
    logger_config.flush_on_write = true;
    logger_config.enable_console_logging = true;
    logger_config.enable_file_logging = true;
    logger_config.enable_json_logging = true;
    logger_config.log_file_path = "/tmp/go2_x5_events.log";
    logger_config.json_log_path = "/tmp/go2_x5_events.jsonl";
    this->event_logger_->Initialize(logger_config);
}

Go2X5Supervisor::WatchdogInput RL_Real_Go2X5::BuildSupervisorInput() const
{
    Go2X5Supervisor::WatchdogInput input;
    input.now_monotonic_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    input.config_loaded = (this->config_loader_ != nullptr);
    input.boot_complete = input.config_loaded;
    input.manifest_valid = this->manifest_valid_;

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        input.body_state_age_us = TimePointAgeUs(this->body_state_stamp_, now);
        input.has_body_state_seq = this->body_state_seen_;
        input.body_state_seq = this->body_state_seq_;
    }
    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        input.arm_state_age_us = TimePointAgeUs(this->arm_bridge_state_stamp, now);
        input.arm_tracking_error_age_us =
            this->arm_tracking_error_high_stamp.time_since_epoch().count() == 0
                ? 0ULL
                : TimePointAgeUs(this->arm_tracking_error_high_stamp, now);
        input.has_arm_state_seq = this->arm_state_seen_;
        input.arm_state_seq = this->arm_state_seq_;
        input.arm_backend_valid = this->arm_bridge_state_valid && this->arm_bridge_state_from_backend;
        input.arm_tracking_error_high = this->arm_tracking_error_high_runtime_;
    }

    input.policy_age_us = TimePointAgeUs(this->last_policy_inference_stamp, now);
    input.has_policy_seq = this->policy_seen_;
    input.policy_seq = this->policy_seq_;

    input.policy_health_bad = this->policy_health_bad_runtime_.load(std::memory_order_relaxed);
    input.policy_health_ok = !input.policy_health_bad;
    input.body_dds_write_ok = this->body_dds_write_ok_runtime_.load(std::memory_order_relaxed);
    input.estop = this->operator_estop_requested_.load(std::memory_order_relaxed);
    input.soft_stop_request = this->arm_safe_shutdown_active.load();
    input.fault_reset = this->operator_fault_reset_requested_.load(std::memory_order_relaxed);
    const bool operator_rl_enabled =
        this->operator_rl_enable_requested_.load(std::memory_order_relaxed) ||
        this->control.navigation_mode;
    input.operator_enable = operator_rl_enabled;
    input.operator_disable = !operator_rl_enabled;
    input.manual_arm_request = this->operator_manual_arm_request_.load(std::memory_order_relaxed);
    const auto& supervisor_config = this->supervisor_->config();
    input.allow_recover = this->manifest_valid_ &&
        input.body_state_age_us <= supervisor_config.body_state_stale_us &&
        input.arm_state_age_us <= supervisor_config.arm_state_stale_us &&
        input.policy_age_us <= supervisor_config.policy_stale_us;
    input.probe_pass = input.manifest_valid &&
        input.has_body_state_seq &&
        input.has_arm_state_seq &&
        input.body_state_age_us <= supervisor_config.body_state_stale_us &&
        input.arm_state_age_us <= supervisor_config.arm_state_stale_us;
    input.probe_fail = !input.manifest_valid;
    return input;
}

rl_sar::runtime::coordinator::Input RL_Real_Go2X5::BuildCoordinatorInput(
    const uint64_t now_monotonic_ns) const
{
    rl_sar::runtime::coordinator::Input input;
    input.now_monotonic_ns = now_monotonic_ns;
    input.mode = Go2X5Supervisor::Mode::Boot;

    {
        std::lock_guard<std::mutex> lock(this->supervisor_mutex);
        if (this->supervisor_)
        {
            input.mode = this->supervisor_->mode();
        }
    }

    const auto now_tp = std::chrono::steady_clock::now();
    const int num_dofs = this->GetNumDofs();
    if (this->robot_state.motor_state.q.size() >= static_cast<size_t>(num_dofs) &&
        this->robot_state.motor_state.dq.size() >= static_cast<size_t>(num_dofs) &&
        this->robot_state.motor_state.tau_est.size() >= static_cast<size_t>(num_dofs))
    {
        input.has_body_state = true;
        auto& frame = input.body_state;
        frame.header.msg_type = rl_sar::protocol::FrameType::BodyState;
        frame.header.seq = this->body_state_seq_;
        frame.header.source_monotonic_ns = now_monotonic_ns;
        frame.header.publish_monotonic_ns = now_monotonic_ns;
        frame.header.mode = static_cast<uint16_t>(input.mode);
        frame.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
        frame.dds_ok = this->body_dds_write_ok_runtime_.load(std::memory_order_relaxed) ? 1U : 0U;
        {
            std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
            frame.header.source_monotonic_ns =
                TimePointToMonotonicNs(this->body_state_stamp_, now_monotonic_ns);
            frame.lowstate_age_us = static_cast<uint32_t>(
                std::min<uint64_t>(TimePointAgeUs(this->body_state_stamp_, now_tp),
                                   std::numeric_limits<uint32_t>::max()));
        }
        for (size_t i = 0; i < std::min<size_t>(4, this->robot_state.imu.quaternion.size()); ++i)
        {
            frame.imu_quat[i] = this->robot_state.imu.quaternion[i];
        }
        for (size_t i = 0; i < std::min<size_t>(3, this->robot_state.imu.gyroscope.size()); ++i)
        {
            frame.imu_gyro[i] = this->robot_state.imu.gyroscope[i];
        }

        size_t body_idx = 0;
        for (int joint_idx = 0;
             joint_idx < num_dofs && body_idx < rl_sar::protocol::kBodyJointCount;
             ++joint_idx)
        {
            if (this->IsArmJointIndex(joint_idx))
            {
                continue;
            }
            const size_t joint_slot = static_cast<size_t>(joint_idx);
            frame.leg_q[body_idx] = this->robot_state.motor_state.q[joint_slot];
            frame.leg_dq[body_idx] = this->robot_state.motor_state.dq[joint_slot];
            frame.leg_tau[body_idx] = this->robot_state.motor_state.tau_est[joint_slot];
            ++body_idx;
        }
    }

    std::vector<float> latest_arm_target;
    uint64_t latest_arm_target_seq = 0;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        latest_arm_target = this->arm_joint_command_latest;
        latest_arm_target_seq = this->arm_joint_command_seq_;
        if (latest_arm_target.size() != static_cast<size_t>(this->arm_joint_count))
        {
            latest_arm_target = this->arm_hold_position;
        }
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        if (this->arm_joint_count > 0 &&
            this->arm_external_state_q.size() >= static_cast<size_t>(this->arm_joint_count))
        {
            input.has_arm_state = true;
            auto& frame = input.arm_state;
            frame.header.msg_type = rl_sar::protocol::FrameType::ArmState;
            frame.header.seq = this->arm_state_seq_;
            frame.header.source_monotonic_ns =
                TimePointToMonotonicNs(this->arm_bridge_state_stamp, now_monotonic_ns);
            frame.header.publish_monotonic_ns = now_monotonic_ns;
            frame.header.mode = static_cast<uint16_t>(input.mode);
            frame.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
            if (this->arm_bridge_state_from_backend)
            {
                frame.header.validity_flags |= rl_sar::protocol::kValidityFromBackend;
            }
            if (!this->arm_bridge_state_from_backend)
            {
                frame.header.validity_flags |= rl_sar::protocol::kValidityShadowState;
            }
            frame.joint_count = static_cast<uint16_t>(this->arm_joint_count);
            frame.backend_age_us = static_cast<uint32_t>(
                std::min<uint64_t>(TimePointAgeUs(this->arm_bridge_state_stamp, now_tp),
                                   std::numeric_limits<uint32_t>::max()));
            frame.transport_age_us = frame.backend_age_us;
            frame.target_seq_applied = latest_arm_target_seq;
            for (int i = 0; i < this->arm_joint_count && i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
            {
                const size_t idx = static_cast<size_t>(i);
                frame.q[idx] = this->arm_external_state_q[idx];
                if (idx < this->arm_external_state_dq.size())
                {
                    frame.dq[idx] = this->arm_external_state_dq[idx];
                }
                if (idx < this->arm_external_state_tau.size())
                {
                    frame.tau[idx] = this->arm_external_state_tau[idx];
                }
                if (idx < latest_arm_target.size())
                {
                    frame.q_target[idx] = latest_arm_target[idx];
                    frame.tracking_error[idx] = latest_arm_target[idx] - this->arm_external_state_q[idx];
                }
            }
        }
        input.arm_backend_valid = this->arm_bridge_state_valid && this->arm_bridge_state_from_backend;
        input.arm_tracking_error_high = this->arm_tracking_error_high_runtime_;
    }

    {
        std::lock_guard<std::mutex> lock(this->policy_frame_mutex);
        if (this->latest_policy_command_valid_)
        {
            input.has_policy_command = true;
            input.dog_policy_command = this->latest_policy_command_frame_;
        }
    }

    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        if (this->arm_joint_command_latest.size() == static_cast<size_t>(this->arm_joint_count) &&
            this->arm_joint_count > 0 &&
            !rl_sar::protocol::IsCommandExpired(
                now_monotonic_ns, this->arm_joint_command_expire_ns_))
        {
            input.has_arm_command = true;
            auto& frame = input.arm_command;
            frame.header.msg_type = rl_sar::protocol::FrameType::ArmCommand;
            frame.header.seq = this->arm_joint_command_seq_;
            frame.header.source_monotonic_ns =
                this->arm_joint_command_source_monotonic_ns_ != 0
                    ? this->arm_joint_command_source_monotonic_ns_
                    : now_monotonic_ns;
            frame.header.publish_monotonic_ns =
                this->arm_joint_command_publish_monotonic_ns_ != 0
                    ? this->arm_joint_command_publish_monotonic_ns_
                    : frame.header.source_monotonic_ns;
            frame.header.mode = static_cast<uint16_t>(input.mode);
            frame.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
            frame.joint_count = static_cast<uint16_t>(this->arm_joint_count);
            frame.command_expire_ns = this->arm_joint_command_expire_ns_;
            const auto fixed_kp = this->GetFixedKp();
            const auto fixed_kd = this->GetFixedKd();
            for (int i = 0; i < this->arm_joint_count && i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
            {
                const size_t idx = static_cast<size_t>(i);
                const int joint_idx = this->arm_joint_start_index + i;
                frame.q[idx] = this->arm_joint_command_latest[idx];
                frame.dq[idx] = 0.0f;
                frame.tau[idx] = 0.0f;
                frame.kp[idx] =
                    (joint_idx >= 0 && joint_idx < static_cast<int>(fixed_kp.size()))
                        ? fixed_kp[static_cast<size_t>(joint_idx)]
                        : 0.0f;
                frame.kd[idx] =
                    (joint_idx >= 0 && joint_idx < static_cast<int>(fixed_kd.size()))
                        ? fixed_kd[static_cast<size_t>(joint_idx)]
                        : 0.0f;
            }
        }
    }

    return input;
}

bool RL_Real_Go2X5::SyncBodyStateFromAdapter(RobotState<float>* state)
{
    if (!state || !this->unitree_adapter_active_ || !this->unitree_adapter_)
    {
        return false;
    }

    rl_sar::protocol::BodyStateFrame frame;
    if (!this->unitree_adapter_->GetState(frame))
    {
        return false;
    }

    const auto diag = this->unitree_adapter_->GetDiagnostics();
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        this->unitree_imu_quaternion = frame.imu_quat;
        this->unitree_imu_gyroscope = frame.imu_gyro;
        for (size_t i = 0; i < rl_sar::protocol::kBodyJointCount; ++i)
        {
            this->unitree_motor_q[i] = frame.leg_q[i];
            this->unitree_motor_dq[i] = frame.leg_dq[i];
            this->unitree_motor_tau[i] = frame.leg_tau[i];
        }
        this->body_state_seen_ = diag.lowstate_seen;
        this->body_state_seq_ = diag.lowstate_seq > 0 ? diag.lowstate_seq : frame.header.seq;
        this->body_state_seq_pending_ = false;
        if (diag.lowstate_age_us != UINT64_MAX)
        {
            this->body_state_stamp_ = now - std::chrono::microseconds(diag.lowstate_age_us);
        }
    }

    for (size_t i = 0; i < std::min<size_t>(4, state->imu.quaternion.size()); ++i)
    {
        state->imu.quaternion[i] = frame.imu_quat[i];
    }
    for (size_t i = 0; i < std::min<size_t>(3, state->imu.gyroscope.size()); ++i)
    {
        state->imu.gyroscope[i] = frame.imu_gyro[i];
    }
    for (size_t i = 0; i < rl_sar::protocol::kBodyJointCount &&
                       i < state->motor_state.q.size() &&
                       i < state->motor_state.dq.size() &&
                       i < state->motor_state.tau_est.size(); ++i)
    {
        state->motor_state.q[i] = frame.leg_q[i];
        state->motor_state.dq[i] = frame.leg_dq[i];
        state->motor_state.tau_est[i] = frame.leg_tau[i];
    }

    this->body_dds_write_ok_runtime_.store(diag.dds_write_ok, std::memory_order_relaxed);
    return true;
}

bool RL_Real_Go2X5::SyncArmStateFromAdapter(RobotState<float>* state)
{
    if (!state || !this->arx_adapter_active_ || !this->arx_adapter_)
    {
        return false;
    }

    rl_sar::protocol::ArmStateFrame frame;
    if (!this->arx_adapter_->GetState(frame))
    {
        return false;
    }

    this->HandleArmBridgeStateFrame(frame, "arx_adapter");
    for (int i = 0; i < this->arm_joint_count &&
                    i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
    {
        const int idx = this->arm_joint_start_index + i;
        if (idx < 0 || idx >= static_cast<int>(state->motor_state.q.size()))
        {
            continue;
        }
        state->motor_state.q[static_cast<size_t>(idx)] = frame.q[static_cast<size_t>(i)];
        state->motor_state.dq[static_cast<size_t>(idx)] = frame.dq[static_cast<size_t>(i)];
        state->motor_state.tau_est[static_cast<size_t>(idx)] = frame.tau[static_cast<size_t>(i)];
    }
    return true;
}

void RL_Real_Go2X5::UpdateRuntimeDiagnostics(const Go2X5Supervisor::TransitionResult* transition_result)
{
    if (!this->diagnostics_publisher_ && !this->drift_recorder_)
    {
        return;
    }

    Go2X5Supervisor::WatchdogStatus watchdog{};
    Go2X5Supervisor::Mode current_mode = Go2X5Supervisor::Mode::Boot;
    uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    if (transition_result)
    {
        watchdog = transition_result->watchdog;
        current_mode = transition_result->current_mode;
        if (transition_result->source_monotonic_ns != 0)
        {
            now_ns = transition_result->source_monotonic_ns;
        }
    }
    else if (this->supervisor_)
    {
        std::lock_guard<std::mutex> lock(this->supervisor_mutex);
        const auto input = this->BuildSupervisorInput();
        watchdog = this->supervisor_->EvaluateWatchdog(input);
        current_mode = this->supervisor_->mode();
        now_ns = input.now_monotonic_ns;
    }

    const auto coordinator_input = this->BuildCoordinatorInput(now_ns);
    if (this->drift_recorder_ && coordinator_input.has_body_state && coordinator_input.has_arm_state)
    {
        this->drift_recorder_->Update(
            coordinator_input.body_state,
            coordinator_input.arm_state,
            rl_sar::diagnostics::CommandContext(
                static_cast<double>(this->control.x),
                static_cast<double>(this->control.y),
                static_cast<double>(this->control.yaw),
                now_ns));
    }

    if (this->diagnostics_publisher_)
    {
        rl_sar::diagnostics::DiagnosticMetrics metrics;
        metrics.timestamp_ns = now_ns;
        metrics.policy_latency_us = watchdog.policy_age_us;
        metrics.policy_frequency_hz = this->last_policy_inference_hz;
        metrics.body_state_age_us = watchdog.body_state_age_us;
        metrics.arm_state_age_us = watchdog.arm_state_age_us;
        metrics.seq_gap_count = static_cast<int>(watchdog.seq_gap_count);
        metrics.current_mode = Go2X5Supervisor::ToString(current_mode);
        metrics.mode_transition_count = this->mode_transition_count_;
        metrics.arm_tracking_healthy = watchdog.arm_backend_valid && !watchdog.arm_tracking_error_high;
        {
            std::lock_guard<std::mutex> metrics_lock(this->runtime_metrics_mutex);
            metrics.coordinator_jitter_us = this->last_coordinator_jitter_us_;
            metrics.coordinator_frequency_hz = this->last_coordinator_frequency_hz_;
        }

        if (coordinator_input.has_arm_state)
        {
            double tracking_error_norm = 0.0;
            for (float value : coordinator_input.arm_state.tracking_error)
            {
                tracking_error_norm += static_cast<double>(value) * static_cast<double>(value);
            }
            metrics.arm_tracking_error_norm = std::sqrt(tracking_error_norm);
        }

        if (this->drift_recorder_)
        {
            const auto snapshots = this->drift_recorder_->GetSnapshots();
            if (!snapshots.empty())
            {
                const auto& latest = snapshots.back();
                metrics.xy_drift_error = latest.xy_drift;
                metrics.yaw_drift_error = latest.yaw_drift;
            }
        }

        this->diagnostics_publisher_->UpdateMetrics(metrics);
    }
}

void RL_Real_Go2X5::LogSupervisorEvent(const Go2X5Supervisor::TransitionResult& result)
{
    if (!this->event_logger_)
    {
        return;
    }

    std::string policy_id;
    if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
    {
        policy_id = this->deploy_manifest_runtime_->Manifest().meta.policy_id;
    }

    double arm_tracking_error = 0.0;
    if (this->arx_adapter_active_ && this->arx_adapter_)
    {
        arm_tracking_error = this->arx_adapter_->GetTrackingError();
    }

    if (result.mode_changed)
    {
        ++this->mode_transition_count_;
        this->event_logger_->LogModeTransition(
            ToLoggerMode(result.previous_mode),
            ToLoggerMode(result.current_mode),
            Go2X5Supervisor::ToString(result.reason_code),
            static_cast<double>(result.watchdog.body_state_age_us),
            static_cast<double>(result.watchdog.arm_state_age_us),
            arm_tracking_error,
            policy_id);
    }

    if (result.reason_code != Go2X5Supervisor::ReasonCode::None)
    {
        this->event_logger_->LogFault(
            ToLoggerFault(result.reason_code),
            Go2X5Supervisor::ToString(result.reason_code),
            static_cast<double>(result.watchdog.body_state_age_us),
            static_cast<double>(result.watchdog.arm_state_age_us),
            arm_tracking_error,
            policy_id,
            result.detail_value);
    }
}

bool RL_Real_Go2X5::ApplyCoordinatorBodyCommand(
    const rl_sar::protocol::BodyCommandFrame& frame,
    RobotCommand<float>* command) const
{
    if (!command || frame.joint_count != rl_sar::protocol::kBodyJointCount)
    {
        return false;
    }

    const int num_dofs = this->GetNumDofs();
    size_t body_idx = 0;
    for (int joint_idx = 0;
         joint_idx < num_dofs && body_idx < rl_sar::protocol::kBodyJointCount;
         ++joint_idx)
    {
        if (this->IsArmJointIndex(joint_idx))
        {
            continue;
        }
        const size_t joint_slot = static_cast<size_t>(joint_idx);
        command->motor_command.q[joint_slot] = frame.q[body_idx];
        command->motor_command.dq[joint_slot] = frame.dq[body_idx];
        command->motor_command.kp[joint_slot] = frame.kp[body_idx];
        command->motor_command.kd[joint_slot] = frame.kd[body_idx];
        command->motor_command.tau[joint_slot] = frame.tau[body_idx];
        ++body_idx;
    }
    return body_idx == rl_sar::protocol::kBodyJointCount;
}

rl_sar::protocol::BodyCommandFrame RL_Real_Go2X5::BuildHoldBodyCommandFrame(
    const uint64_t now_monotonic_ns,
    const Go2X5Supervisor::Mode mode) const
{
    rl_sar::protocol::BodyCommandFrame frame;
    frame.header.msg_type = rl_sar::protocol::FrameType::BodyCommand;
    frame.header.source_monotonic_ns = now_monotonic_ns;
    frame.header.publish_monotonic_ns = now_monotonic_ns;
    frame.header.mode = static_cast<uint16_t>(mode);
    frame.header.validity_flags =
        rl_sar::protocol::kValidityPayloadValid | rl_sar::protocol::kValidityFallbackGenerated;
    frame.joint_count = rl_sar::protocol::kBodyJointCount;
    frame.command_expire_ns = now_monotonic_ns + 15'000'000ULL;
    if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
    {
        const auto snapshot = this->deploy_manifest_runtime_->Snapshot();
        if (snapshot.body_command_expire_ms > 0)
        {
            frame.command_expire_ns = now_monotonic_ns +
                static_cast<uint64_t>(snapshot.body_command_expire_ms) * 1'000'000ULL;
        }
    }

    if (IsPassiveBodyOutputMode(mode))
    {
        for (size_t i = 0; i < rl_sar::protocol::kBodyJointCount; ++i)
        {
            frame.q[i] = static_cast<float>(PosStopF);
            frame.dq[i] = static_cast<float>(VelStopF);
            frame.kp[i] = 0.0f;
            frame.kd[i] = 0.0f;
            frame.tau[i] = 0.0f;
        }
        return frame;
    }

    const auto default_dof_pos = this->GetDefaultDofPos();
    const auto fixed_kp = this->GetFixedKp();
    const auto fixed_kd = this->GetFixedKd();
    const int num_dofs = this->GetNumDofs();
    size_t body_idx = 0;
    for (int joint_idx = 0;
         joint_idx < num_dofs && body_idx < rl_sar::protocol::kBodyJointCount;
         ++joint_idx)
    {
        if (this->IsArmJointIndex(joint_idx))
        {
            continue;
        }

        const size_t joint_slot = static_cast<size_t>(joint_idx);
        const float q_ref =
            joint_slot < default_dof_pos.size() ? default_dof_pos[joint_slot] : 0.0f;
        const float kp =
            joint_slot < fixed_kp.size() ? fixed_kp[joint_slot] : 0.0f;
        const float kd =
            joint_slot < fixed_kd.size() ? fixed_kd[joint_slot] : 0.0f;

        frame.q[body_idx] = q_ref;
        frame.dq[body_idx] = 0.0f;
        frame.kp[body_idx] = kp;
        frame.kd[body_idx] = kd;

        float tau = 0.0f;
        if (joint_slot < this->robot_state.motor_state.q.size() &&
            joint_slot < this->robot_state.motor_state.dq.size())
        {
            tau = kp * (q_ref - this->robot_state.motor_state.q[joint_slot]) -
                  kd * this->robot_state.motor_state.dq[joint_slot];
        }
        if (joint_slot < this->whole_body_effort_limits.size() &&
            std::isfinite(this->whole_body_effort_limits[joint_slot]) &&
            this->whole_body_effort_limits[joint_slot] > 0.0f)
        {
            const float limit = std::fabs(this->whole_body_effort_limits[joint_slot]);
            tau = std::max(-limit, std::min(limit, tau));
        }
        frame.tau[body_idx] = tau;
        ++body_idx;
    }

    return frame;
}

rl_sar::protocol::ArmCommandFrame RL_Real_Go2X5::BuildHoldArmCommandFrame(
    const uint64_t now_monotonic_ns,
    const Go2X5Supervisor::Mode mode) const
{
    rl_sar::protocol::ArmCommandFrame frame;
    frame.header.msg_type = rl_sar::protocol::FrameType::ArmCommand;
    frame.header.source_monotonic_ns = now_monotonic_ns;
    frame.header.publish_monotonic_ns = now_monotonic_ns;
    frame.header.mode = static_cast<uint16_t>(mode);
    frame.header.validity_flags =
        rl_sar::protocol::kValidityPayloadValid | rl_sar::protocol::kValidityFallbackGenerated;
    frame.joint_count = static_cast<uint16_t>(this->arm_joint_count);
    frame.command_expire_ns = now_monotonic_ns + 15'000'000ULL;
    if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
    {
        const auto snapshot = this->deploy_manifest_runtime_->Snapshot();
        if (snapshot.arm_command_expire_ms > 0)
        {
            frame.command_expire_ns = now_monotonic_ns +
                static_cast<uint64_t>(snapshot.arm_command_expire_ms) * 1'000'000ULL;
        }
    }

    std::vector<float> hold_position;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        hold_position = this->arm_hold_position;
    }
    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        if (this->arm_external_state_q.size() >= static_cast<size_t>(this->arm_joint_count))
        {
            hold_position = this->arm_external_state_q;
        }
    }

    const auto fixed_kp = this->GetFixedKp();
    const auto fixed_kd = this->GetFixedKd();
    for (int i = 0; i < this->arm_joint_count && i < static_cast<int>(rl_sar::protocol::kArmJointCount); ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        const int joint_idx = this->arm_joint_start_index + i;
        frame.q[idx] = idx < hold_position.size() ? hold_position[idx] : 0.0f;
        frame.dq[idx] = 0.0f;
        frame.tau[idx] = 0.0f;
        frame.kp[idx] =
            (joint_idx >= 0 && joint_idx < static_cast<int>(fixed_kp.size()))
                ? fixed_kp[static_cast<size_t>(joint_idx)]
                : 0.0f;
        frame.kd[idx] =
            (joint_idx >= 0 && joint_idx < static_cast<int>(fixed_kd.size()))
                ? fixed_kd[static_cast<size_t>(joint_idx)]
                : 0.0f;
    }
    return frame;
}

void RL_Real_Go2X5::RefreshSupervisorState(const char* source)
{
    if (!this->supervisor_)
    {
        return;
    }

    std::lock_guard<std::mutex> supervisor_lock(this->supervisor_mutex);

    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        if (this->body_state_seq_pending_)
        {
            ++this->body_state_seq_;
            this->body_state_seq_pending_ = false;
        }
    }
    {
        std::lock_guard<std::mutex> lock(this->arm_external_state_mutex);
        if (this->arm_state_seq_pending_)
        {
            ++this->arm_state_seq_;
            this->arm_state_seq_pending_ = false;
        }
    }

    const auto input = this->BuildSupervisorInput();
    const auto result = this->supervisor_->Step(input);

    if (this->state_manager_)
    {
        this->state_manager_->SetArmBridgeConnected(
            input.arm_backend_valid &&
            input.arm_state_age_us <= this->supervisor_->config().arm_state_stale_us);
    }

    if (result.mode_changed)
    {
        std::cout << LOGGER::INFO
                  << "[Supervisor] " << source << ": "
                  << Go2X5Supervisor::ToString(result.previous_mode)
                  << " -> " << Go2X5Supervisor::ToString(result.current_mode)
                  << " reason=" << Go2X5Supervisor::ToString(result.reason_code)
                  << " manifest_valid=" << (input.manifest_valid ? "true" : "false")
                  << " body_age_us=" << result.watchdog.body_state_age_us
                  << " arm_age_us=" << result.watchdog.arm_state_age_us
                  << " policy_age_us=" << result.watchdog.policy_age_us
                  << std::endl;
    }

    this->LogSupervisorEvent(result);
    this->UpdateRuntimeDiagnostics(&result);
}

int RL_Real_Go2X5::GetNumDofs() const
{
    return config_->GetNumDofs();
}

float RL_Real_Go2X5::GetDt() const
{
    return config_->GetDt();
}

int RL_Real_Go2X5::GetDecimation() const
{
    return config_->GetDecimation();
}

std::vector<float> RL_Real_Go2X5::GetDefaultDofPos() const
{
    return config_->GetDefaultDofPos();
}

std::vector<float> RL_Real_Go2X5::GetFixedKp() const
{
    return config_->GetFixedKp();
}

std::vector<float> RL_Real_Go2X5::GetFixedKd() const
{
    return config_->GetFixedKd();
}

std::vector<int> RL_Real_Go2X5::GetJointMapping() const
{
    return config_->GetJointMapping();
}

// Safe shutdown functions moved to go2_x5_safe_shutdown.cpp
// - BuildSafeShutdownTargetPose
// - PublishWholeBodyPose
// - ExecuteSafeShutdownSequence
// State capture/apply functions moved to go2_x5_state_management.cpp
// - CaptureArmCommandStateLocked
// - ApplyArmCommandStateLocked
// - CaptureArmBridgeRuntimeStateLocked
// - ApplyArmBridgeRuntimeStateLocked

void RL_Real_Go2X5::InitializeArmCommandState()
{
    std::lock_guard<std::mutex> lock(this->arm_command_mutex);
    this->cmd_vel_alpha = config_->GetCmdVelAlpha();
    this->joystick_deadband = std::max(0.0f, config_->GetJoystickDeadband());
    this->arm_joint_command_topic = config_->GetArmJointCommandTopic();
    const bool key2_prefer_topic_command = config_->GetKey2PreferTopicCommand();
    if (!key2_prefer_topic_command && !this->arm_joint_command_topic.empty())
    {
        std::cout << LOGGER::WARNING
                  << "key2_prefer_topic_command=false. Key[2] will ignore the latest "
                  << this->arm_joint_command_topic << " target and use arm_key_pose/arm_hold_pose."
                  << std::endl;
    }

    Go2X5ArmRuntime::InitializationConfig config;
    config.arm_command_size = config_->GetArmCommandSize();
    config.arm_joint_start_index = this->arm_joint_start_index;
    config.arm_hold_enabled = config_->GetArmHoldEnabled();
    const float dt = GetDt();
    const int decimation = GetDecimation();
    config.step_dt = dt * decimation;
    config.smoothing_time = config_->GetArmCommandSmoothingTime();
    config.arm_hold_pose = config_->GetArmHoldPose();
    config.default_dof_pos = GetDefaultDofPos();
    this->ApplyArmCommandStateLocked(Go2X5ArmRuntime::BuildInitialCommandState(config));

    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    uint64_t expire_ns = now_ns + 15'000'000ULL;
    if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
    {
        const auto snapshot = this->deploy_manifest_runtime_->Snapshot();
        if (snapshot.arm_command_expire_ms > 0)
        {
            expire_ns = now_ns + static_cast<uint64_t>(snapshot.arm_command_expire_ms) * 1'000'000ULL;
        }
    }
    this->arm_joint_command_source_monotonic_ns_ = now_ns;
    this->arm_joint_command_publish_monotonic_ns_ = now_ns;
    this->arm_joint_command_expire_ns_ = expire_ns;
    ++this->arm_joint_command_seq_;
}

void RL_Real_Go2X5::InitializeRealDeploySafetyConfig()
{
    const int num_dofs = GetNumDofs();
    this->real_deploy_exclusive_keyboard_control = config_->GetRealDeployExclusiveKeyboardControl();
    this->policy_inference_log_enabled = config_->GetPolicyInferenceLogEnabled();
    this->cmd_vel_input_ignored_warned = false;
    this->last_policy_inference_hz = 0.0f;
    this->last_policy_inference_stamp = std::chrono::steady_clock::time_point{};

    auto lower_limits = config_->GetJointLowerLimits();
    if (lower_limits.size() != static_cast<size_t>(num_dofs))
    {
        lower_limits = this->GetDefaultWholeBodyLowerLimits();
    }
    if (lower_limits.size() != static_cast<size_t>(num_dofs))
    {
        lower_limits.assign(static_cast<size_t>(num_dofs), -std::numeric_limits<float>::infinity());
    }
    this->whole_body_joint_lower_limits = std::move(lower_limits);

    auto upper_limits = config_->GetJointUpperLimits();
    if (upper_limits.size() != static_cast<size_t>(num_dofs))
    {
        upper_limits = this->GetDefaultWholeBodyUpperLimits();
    }
    if (upper_limits.size() != static_cast<size_t>(num_dofs))
    {
        upper_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_joint_upper_limits = std::move(upper_limits);

    auto velocity_limits = config_->GetJointVelocityLimits();
    if (velocity_limits.size() != static_cast<size_t>(num_dofs))
    {
        velocity_limits = this->GetDefaultWholeBodyVelocityLimits();
    }
    if (velocity_limits.size() != static_cast<size_t>(num_dofs))
    {
        velocity_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_velocity_limits = std::move(velocity_limits);

    auto effort_limits = config_->GetJointEffortLimits();
    if (effort_limits.size() != static_cast<size_t>(num_dofs))
    {
        effort_limits = this->GetDefaultWholeBodyEffortLimits();
    }
    if (effort_limits.size() != static_cast<size_t>(num_dofs))
    {
        effort_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_effort_limits = std::move(effort_limits);

    auto kp_limits = config_->GetJointKpLimits();
    if (kp_limits.size() != static_cast<size_t>(num_dofs))
    {
        kp_limits = this->GetDefaultWholeBodyKpLimits();
    }
    if (kp_limits.size() != static_cast<size_t>(num_dofs))
    {
        kp_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_kp_limits = std::move(kp_limits);

    auto kd_limits = config_->GetJointKdLimits();
    if (kd_limits.size() != static_cast<size_t>(num_dofs))
    {
        kd_limits = this->GetDefaultWholeBodyKdLimits();
    }
    if (kd_limits.size() != static_cast<size_t>(num_dofs))
    {
        kd_limits.assign(static_cast<size_t>(num_dofs), std::numeric_limits<float>::infinity());
    }
    this->whole_body_kd_limits = std::move(kd_limits);
}

// CaptureArmRuntimeStateLocked and RestoreArmRuntimeStateLocked(overload) moved to go2_x5_state_management.cpp

void RL_Real_Go2X5::InitializeArmChannelConfig()
{
    const int num_dofs = GetNumDofs();
    this->arm_joint_count = config_->GetArmJointCount();
    this->arm_joint_start_index = config_->GetArmJointStartIndex();

    if (this->arm_joint_start_index < 0)
    {
        this->arm_joint_start_index = 0;
    }
    if (this->arm_joint_start_index > num_dofs)
    {
        this->arm_joint_start_index = num_dofs;
    }
    if (this->arm_joint_count < 0)
    {
        this->arm_joint_count = 0;
    }
    if (this->arm_joint_start_index + this->arm_joint_count > num_dofs)
    {
        this->arm_joint_count = std::max(0, num_dofs - this->arm_joint_start_index);
    }

    this->arm_control_mode = config_->GetArmControlMode();
    std::transform(this->arm_control_mode.begin(), this->arm_control_mode.end(), this->arm_control_mode.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    this->arm_split_control_enabled = config_->IsArmSplitControlEnabled();

#if !defined(USE_ARX_X5_SDK)
    if (this->arm_control_mode == "arx_x5")
    {
        std::cout << LOGGER::WARNING
                  << "arm_control_mode=arx_x5 but binary was built without USE_ARX_X5_SDK. "
                  << "Falling back to bridge transport." << std::endl;
    }
#endif

    this->arm_bridge_cmd_topic = config_->GetArmBridgeCmdTopic();
    this->arm_bridge_state_topic = config_->GetArmBridgeStateTopic();
    const bool arm_bridge_require_state = config_->GetArmBridgeRequireState();
    this->arm_bridge_require_state = arm_bridge_require_state;
    this->arm_bridge_require_live_state = config_->GetArmBridgeRequireLiveState();
    this->arm_bridge_shadow_feedback_enabled = config_->GetArmBridgeShadowFeedbackEnabled();
    this->arm_bridge_state_timeout_sec = config_->GetArmBridgeStateTimeoutSec();
    if (this->arm_bridge_state_timeout_sec < 0.0f)
    {
        this->arm_bridge_state_timeout_sec = 0.0f;
    }
    {
        std::vector<float> hold_position_local;
        int arm_command_size_local = 0;
        {
            std::lock_guard<std::mutex> command_lock(this->arm_command_mutex);
            hold_position_local = this->arm_hold_position;
            arm_command_size_local = this->arm_command_size;
        }
        std::lock_guard<std::mutex> state_lock(this->arm_external_state_mutex);
        auto bridge_state = this->CaptureArmBridgeRuntimeStateLocked();
        Go2X5ArmBridgeRuntime::InitializationConfig config;
        config.joint_count = this->arm_joint_count;
        config.command_size = arm_command_size_local;
        config.require_state = this->arm_bridge_require_state;
        config.require_live_state = this->arm_bridge_require_live_state;
        config.state_timeout_sec = this->arm_bridge_state_timeout_sec;
        config.hold_position = hold_position_local;
        Go2X5ArmBridgeRuntime::ReconcileConfiguration(&bridge_state, config);
        this->ApplyArmBridgeRuntimeStateLocked(bridge_state);
    }

    auto lower_limits = config_->GetArmJointLowerLimits();
    if (lower_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        lower_limits = this->GetDefaultArmLowerLimits();
    }
    if (lower_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        lower_limits.assign(static_cast<size_t>(this->arm_joint_count), -std::numeric_limits<float>::infinity());
    }
    this->arm_joint_lower_limits = std::move(lower_limits);

    auto upper_limits = config_->GetArmJointUpperLimits();
    if (upper_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        upper_limits = this->GetDefaultArmUpperLimits();
    }
    if (upper_limits.size() != static_cast<size_t>(this->arm_joint_count))
    {
        upper_limits.assign(static_cast<size_t>(this->arm_joint_count), std::numeric_limits<float>::infinity());
    }
    this->arm_joint_upper_limits = std::move(upper_limits);

    std::cout << LOGGER::INFO << "Arm control mode: " << this->arm_control_mode
              << " (start=" << this->arm_joint_start_index
              << ", count=" << static_cast<int>(this->arm_joint_count) << ")" << std::endl;
    if (this->arm_split_control_enabled)
    {
        std::cout << LOGGER::INFO << "Arm bridge guard: require_state="
                  << (this->arm_bridge_require_state ? "true" : "false")
                  << ", require_live_state=" << (this->arm_bridge_require_live_state ? "true" : "false")
                  << ", shadow_feedback=" << (this->arm_bridge_shadow_feedback_enabled ? "true" : "false")
                  << ", timeout=" << this->arm_bridge_state_timeout_sec << "s" << std::endl;
    }
}

// Utility functions moved to go2_x5_utility.cpp
// - UseExclusiveRealDeployControl
// - UseArmBridgeIpc

// Limits functions moved to go2_x5_limits.cpp

bool RL_Real_Go2X5::ClipWholeBodyCommand(RobotCommand<float> *command, const char* context) const
{
    if (!command)
    {
        return false;
    }

    const int num_dofs = GetNumDofs();
    if (num_dofs <= 0)
    {
        return false;
    }

    const auto default_pos = GetDefaultDofPos();
    const auto fixed_kp = GetFixedKp();
    const auto fixed_kd = GetFixedKd();
    const auto joint_names = config_->GetJointNames();

    int clipped_count = 0;
    std::string first_joint;
    for (int i = 0; i < num_dofs; ++i)
    {
        const float q_fallback =
            (i < static_cast<int>(default_pos.size())) ? default_pos[static_cast<size_t>(i)] : 0.0f;
        const float dq_fallback = 0.0f;
        const float kp_fallback =
            (i < static_cast<int>(fixed_kp.size())) ? fixed_kp[static_cast<size_t>(i)] : 0.0f;
        const float kd_fallback =
            (i < static_cast<int>(fixed_kd.size())) ? fixed_kd[static_cast<size_t>(i)] : 0.0f;
        const float q_lo =
            (i < static_cast<int>(this->whole_body_joint_lower_limits.size()))
                ? this->whole_body_joint_lower_limits[static_cast<size_t>(i)]
                : -std::numeric_limits<float>::infinity();
        const float q_hi =
            (i < static_cast<int>(this->whole_body_joint_upper_limits.size()))
                ? this->whole_body_joint_upper_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float dq_limit =
            (i < static_cast<int>(this->whole_body_velocity_limits.size()))
                ? this->whole_body_velocity_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float tau_limit =
            (i < static_cast<int>(this->whole_body_effort_limits.size()))
                ? this->whole_body_effort_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float kp_limit =
            (i < static_cast<int>(this->whole_body_kp_limits.size()))
                ? this->whole_body_kp_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();
        const float kd_limit =
            (i < static_cast<int>(this->whole_body_kd_limits.size()))
                ? this->whole_body_kd_limits[static_cast<size_t>(i)]
                : std::numeric_limits<float>::infinity();

        auto clip_position = [&](float value) -> float
        {
            float sanitized = std::isfinite(value) ? value : q_fallback;
            if (std::isfinite(q_lo) && std::isfinite(q_hi))
            {
                const float lo = std::min(q_lo, q_hi);
                const float hi = std::max(q_lo, q_hi);
                sanitized = std::clamp(sanitized, lo, hi);
            }
            return sanitized;
        };
        auto clip_abs = [](float value, float fallback, float abs_limit, bool non_negative) -> float
        {
            float sanitized = std::isfinite(value) ? value : fallback;
            if (std::isfinite(abs_limit))
            {
                const float lo = non_negative ? 0.0f : -std::fabs(abs_limit);
                const float hi = std::fabs(abs_limit);
                sanitized = std::clamp(sanitized, lo, hi);
            }
            return sanitized;
        };

        const float old_q = command->motor_command.q[static_cast<size_t>(i)];
        const float old_dq = command->motor_command.dq[static_cast<size_t>(i)];
        const float old_tau = command->motor_command.tau[static_cast<size_t>(i)];
        const float old_kp = command->motor_command.kp[static_cast<size_t>(i)];
        const float old_kd = command->motor_command.kd[static_cast<size_t>(i)];

        command->motor_command.q[static_cast<size_t>(i)] = clip_position(old_q);
        command->motor_command.dq[static_cast<size_t>(i)] = clip_abs(old_dq, dq_fallback, dq_limit, false);
        command->motor_command.tau[static_cast<size_t>(i)] = clip_abs(old_tau, 0.0f, tau_limit, false);
        command->motor_command.kp[static_cast<size_t>(i)] = clip_abs(old_kp, kp_fallback, kp_limit, true);
        command->motor_command.kd[static_cast<size_t>(i)] = clip_abs(old_kd, kd_fallback, kd_limit, true);

        const bool joint_clipped =
            !std::isfinite(old_q) || !std::isfinite(old_dq) || !std::isfinite(old_tau) ||
            !std::isfinite(old_kp) || !std::isfinite(old_kd) ||
            std::fabs(command->motor_command.q[static_cast<size_t>(i)] - old_q) > 1e-6f ||
            std::fabs(command->motor_command.dq[static_cast<size_t>(i)] - old_dq) > 1e-6f ||
            std::fabs(command->motor_command.tau[static_cast<size_t>(i)] - old_tau) > 1e-6f ||
            std::fabs(command->motor_command.kp[static_cast<size_t>(i)] - old_kp) > 1e-6f ||
            std::fabs(command->motor_command.kd[static_cast<size_t>(i)] - old_kd) > 1e-6f;
        if (joint_clipped)
        {
            ++clipped_count;
            if (first_joint.empty())
            {
                first_joint = (i < static_cast<int>(joint_names.size())) ? joint_names[static_cast<size_t>(i)]
                                                                         : std::to_string(i);
            }
        }
    }

    if (clipped_count > 0)
    {
        const auto now = std::chrono::steady_clock::now();
        const bool should_log =
            this->whole_body_clip_warn_stamp.time_since_epoch().count() == 0 ||
            std::chrono::duration_cast<std::chrono::milliseconds>(now - this->whole_body_clip_warn_stamp).count() >= 1000;
        if (should_log)
        {
            this->whole_body_clip_warn_stamp = now;
            std::cout << LOGGER::WARNING << context << " clipped " << static_cast<int>(clipped_count)
                      << " joint command(s), first=" << first_joint << std::endl;
        }
        return true;
    }
    return false;
}

// Clipping and validation functions moved to go2_x5_clip_validation.cpp
// IsArmBridgeStateFreshLocked moved to go2_x5_state_management.cpp

void RL_Real_Go2X5::SetupArmCommandSubscriber()
{
    this->CloseArmCommandIpc();
    this->SetupArmCommandIpc();
#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (this->arm_joint_command_topic.empty())
    {
        return;
    }
    this->arm_joint_command_topic_active = this->arm_joint_command_topic;
    std::cout << LOGGER::INFO
              << "ROS Float32 arm command fallback disabled. "
              << "Expecting typed IPC for " << this->arm_joint_command_topic_active << "."
              << std::endl;
#endif
}

// IPC functions moved to go2_x5_ipc.cpp
// - SetupArmCommandIpc
// - CloseArmCommandIpc
// - PollArmCommandIpc
// - SetupArmBridgeIpc
// - CloseArmBridgeIpc
// - PollArmBridgeIpcState
// - SetupArmBridgeInterface

// - IsArmJointIndex
// - IsInRLLocomotionState

// Arm limits functions moved to go2_x5_limits.cpp
// Arm validation functions moved to go2_x5_clip_validation.cpp

// State I/O functions moved to go2_x5_state_io.cpp
// - ReadArmStateFromExternal
// - WriteArmCommandToExternal
// - ApplyArmHold
// - ArmCommandDifferent

void RL_Real_Go2X5::GetState(RobotState<float> *state)
{
    this->SyncBodyStateFromAdapter(state);
    std::array<float, 4> imu_quat{};
    std::array<float, 3> imu_gyro{};
    std::array<float, 20> motor_q{};
    std::array<float, 20> motor_dq{};
    std::array<float, 20> motor_tau{};
    float joy_lx = 0.0f;
    float joy_ly = 0.0f;
    float joy_rx = 0.0f;
    xKeySwitchUnion joy_bits;
    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        imu_quat = this->unitree_imu_quaternion;
        imu_gyro = this->unitree_imu_gyroscope;
        motor_q = this->unitree_motor_q;
        motor_dq = this->unitree_motor_dq;
        motor_tau = this->unitree_motor_tau;
        joy_lx = this->unitree_joy_lx;
        joy_ly = this->unitree_joy_ly;
        joy_rx = this->unitree_joy_rx;
        joy_bits = this->unitree_joy;
    }

    if (!this->UseExclusiveRealDeployControl())
    {
        if (joy_bits.components.A) this->control.SetGamepad(Input::Gamepad::A);
        if (joy_bits.components.B) this->control.SetGamepad(Input::Gamepad::B);
        if (joy_bits.components.X) this->control.SetGamepad(Input::Gamepad::X);
        if (joy_bits.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
        if (joy_bits.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
        if (joy_bits.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
        if (joy_bits.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
        if (joy_bits.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
        if (joy_bits.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
        if (joy_bits.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
        if (joy_bits.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
        if (joy_bits.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
        if (joy_bits.components.L1 && joy_bits.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
        if (joy_bits.components.L1 && joy_bits.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
        if (joy_bits.components.L1 && joy_bits.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
        if (joy_bits.components.L1 && joy_bits.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
        if (joy_bits.components.L1 && joy_bits.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
        if (joy_bits.components.L1 && joy_bits.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
        if (joy_bits.components.L1 && joy_bits.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
        if (joy_bits.components.L1 && joy_bits.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
        if (joy_bits.components.L1 && joy_bits.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
        if (joy_bits.components.L1 && joy_bits.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
        if (joy_bits.components.R1 && joy_bits.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
        if (joy_bits.components.R1 && joy_bits.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
        if (joy_bits.components.R1 && joy_bits.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
        if (joy_bits.components.R1 && joy_bits.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
        if (joy_bits.components.R1 && joy_bits.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
        if (joy_bits.components.R1 && joy_bits.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
        if (joy_bits.components.R1 && joy_bits.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
        if (joy_bits.components.R1 && joy_bits.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
        if (joy_bits.components.R1 && joy_bits.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
        if (joy_bits.components.R1 && joy_bits.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
        if (joy_bits.components.L1 && joy_bits.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

        const bool fixed_cmd_latched =
            (this->control.current_keyboard == Input::Keyboard::Num1 &&
             this->control.last_keyboard == Input::Keyboard::Num1);
        const bool joystick_active =
            (std::fabs(joy_ly) > this->joystick_deadband) ||
            (std::fabs(joy_lx) > this->joystick_deadband) ||
            (std::fabs(joy_rx) > this->joystick_deadband);
        if (!fixed_cmd_latched || joystick_active)
        {
            this->control.x = joy_ly;
            this->control.y = -joy_lx;
            this->control.yaw = -joy_rx;
        }

#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            std::lock_guard<std::mutex> cmd_lock(this->cmd_vel_mutex);
            if (this->cmd_vel_has_filtered)
            {
                this->control.x = static_cast<float>(this->cmd_vel_filtered.linear.x);
                this->control.y = static_cast<float>(this->cmd_vel_filtered.linear.y);
                this->control.yaw = static_cast<float>(this->cmd_vel_filtered.angular.z);
            }
        }
#endif
    }

    state->imu.quaternion[0] = imu_quat[0]; // w
    state->imu.quaternion[1] = imu_quat[1]; // x
    state->imu.quaternion[2] = imu_quat[2]; // y
    state->imu.quaternion[3] = imu_quat[3]; // z

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = imu_gyro[static_cast<size_t>(i)];
    }
    const int num_dofs = GetNumDofs();
    const auto joint_mapping = GetJointMapping();
    for (int i = 0; i < num_dofs; ++i)
    {
        const int mapped = joint_mapping[i];
        if (mapped >= 0 && mapped < static_cast<int>(motor_q.size()))
        {
            state->motor_state.q[i] = motor_q[static_cast<size_t>(mapped)];
            state->motor_state.dq[i] = motor_dq[static_cast<size_t>(mapped)];
            state->motor_state.tau_est[i] = motor_tau[static_cast<size_t>(mapped)];
        }
        else
        {
            state->motor_state.q[i] = 0.0f;
            state->motor_state.dq[i] = 0.0f;
            state->motor_state.tau_est[i] = 0.0f;
        }
    }

    this->ReadArmStateFromExternal(state);
}

void RL_Real_Go2X5::SetCommand(const RobotCommand<float> *command)
{
    RobotCommand<float> command_local = *command;
    this->ClipWholeBodyCommand(&command_local, "Real deploy whole-body command");

    const uint64_t now_monotonic_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    Go2X5Supervisor::Mode supervisor_mode = Go2X5Supervisor::Mode::Boot;
    rl_sar::protocol::BodyCommandFrame final_body_command =
        this->BuildHoldBodyCommandFrame(now_monotonic_ns, supervisor_mode);
    rl_sar::protocol::ArmCommandFrame final_arm_command =
        this->BuildHoldArmCommandFrame(now_monotonic_ns, supervisor_mode);
    if (this->coordinator_)
    {
        const auto coordinator_input = this->BuildCoordinatorInput(now_monotonic_ns);
        supervisor_mode = coordinator_input.mode;
        final_body_command = this->BuildHoldBodyCommandFrame(now_monotonic_ns, supervisor_mode);
        final_arm_command = this->BuildHoldArmCommandFrame(now_monotonic_ns, supervisor_mode);
        const auto coordinator_output = this->coordinator_->Step(coordinator_input);
        {
            std::lock_guard<std::mutex> metrics_lock(this->runtime_metrics_mutex);
            const auto now_tick = std::chrono::steady_clock::now();
            if (this->last_coordinator_tick_.time_since_epoch().count() != 0)
            {
                const auto delta_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        now_tick - this->last_coordinator_tick_).count());
                int expected_rate_hz = 200;
                if (this->deploy_manifest_runtime_ && this->deploy_manifest_runtime_->HasManifest())
                {
                    expected_rate_hz = std::max(1, this->deploy_manifest_runtime_->Snapshot().coordinator_rate_hz);
                }
                const uint64_t expected_period_us = static_cast<uint64_t>(1'000'000ULL / std::max(1, expected_rate_hz));
                this->last_coordinator_jitter_us_ =
                    delta_us > expected_period_us ? (delta_us - expected_period_us) : (expected_period_us - delta_us);
                this->last_coordinator_frequency_hz_ =
                    delta_us > 0 ? (1'000'000.0 / static_cast<double>(delta_us)) : 0.0;
            }
            this->last_coordinator_tick_ = now_tick;
        }
        if (coordinator_output.body_command_valid)
        {
            final_body_command = coordinator_output.body_command;
        }
        if (coordinator_output.arm_command_valid)
        {
            final_arm_command = coordinator_output.arm_command;
        }
    }
    this->ApplyCoordinatorBodyCommand(final_body_command, &command_local);

    const int num_dofs = GetNumDofs();
    const auto joint_mapping = GetJointMapping();
    for (int i = 0; i < num_dofs; ++i)
    {
        if (i >= static_cast<int>(joint_mapping.size()))
        {
            continue;
        }
        const int mapped = joint_mapping[static_cast<size_t>(i)];
        if (mapped < 0 || mapped >= 20)
        {
            continue;
        }

        if (this->arm_split_control_enabled && this->IsArmJointIndex(i))
        {
            // Arm joints are controlled by an external channel in split mode.
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x00;
            this->unitree_low_command.motor_cmd()[mapped].q() = PosStopF;
            this->unitree_low_command.motor_cmd()[mapped].dq() = VelStopF;
            this->unitree_low_command.motor_cmd()[mapped].kp() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].kd() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
        }
        else if (IsPassiveBodyJointCommand(command_local.motor_command.q[i],
                                           command_local.motor_command.dq[i],
                                           command_local.motor_command.kp[i],
                                           command_local.motor_command.kd[i],
                                           command_local.motor_command.tau[i]))
        {
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x00;
            this->unitree_low_command.motor_cmd()[mapped].q() = PosStopF;
            this->unitree_low_command.motor_cmd()[mapped].dq() = VelStopF;
            this->unitree_low_command.motor_cmd()[mapped].kp() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].kd() = 0.0f;
            this->unitree_low_command.motor_cmd()[mapped].tau() = 0.0f;
        }
        else
        {
            this->unitree_low_command.motor_cmd()[mapped].mode() = 0x01;
            this->unitree_low_command.motor_cmd()[mapped].q() = command_local.motor_command.q[i];
            this->unitree_low_command.motor_cmd()[mapped].dq() = command_local.motor_command.dq[i];
            this->unitree_low_command.motor_cmd()[mapped].kp() = command_local.motor_command.kp[i];
            this->unitree_low_command.motor_cmd()[mapped].kd() = command_local.motor_command.kd[i];
            this->unitree_low_command.motor_cmd()[mapped].tau() = command_local.motor_command.tau[i];
        }
    }

    this->WriteArmCommandFrameToExternal(final_arm_command);

    bool body_dds_write_ok = false;
    if (this->unitree_adapter_active_ && this->unitree_adapter_)
    {
        const auto set_status = this->unitree_adapter_->SetCommand(final_body_command);
        const auto process_status =
            (set_status == rl_sar::adapters::UnitreeAdapter::Status::kOk)
                ? this->unitree_adapter_->ProcessCommand()
                : set_status;
        body_dds_write_ok = (process_status == rl_sar::adapters::UnitreeAdapter::Status::kOk);
        if (!body_dds_write_ok)
        {
            std::cout << LOGGER::WARNING
                      << "[SupervisorInput] source=unitree_adapter reason=body_command_write_failed"
                      << std::endl;
        }
    }
    else
    {
        this->unitree_low_command.crc() =
            Crc32Core((uint32_t *)&unitree_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        try
        {
            if (lowcmd_publisher)
            {
                lowcmd_publisher->Write(unitree_low_command);
                body_dds_write_ok = true;
            }
            else
            {
                std::cout << LOGGER::WARNING
                          << "[SupervisorInput] source=lowcmd_write reason=publisher_unavailable"
                          << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::WARNING
                      << "[SupervisorInput] source=lowcmd_write reason=publish_exception: "
                      << e.what() << std::endl;
        }
    }

    this->body_dds_write_ok_runtime_.store(body_dds_write_ok, std::memory_order_relaxed);
    this->RefreshSupervisorState("lowcmd_write");
}

void RL_Real_Go2X5::RobotControl()
{
    this->GetState(&this->robot_state);
    this->PollArmCommandIpc();

    this->StateController(&this->robot_state, &this->robot_command);
    this->MaybePublishKey1CmdVel();

    bool operator_input_triggered = false;
    if (this->control.current_keyboard == Input::Keyboard::Escape)
    {
        this->operator_estop_requested_.store(true, std::memory_order_relaxed);
        this->operator_rl_enable_requested_.store(false, std::memory_order_relaxed);
        std::cout << LOGGER::WARNING
                  << "[SupervisorInput] source=keyboard:Escape reason=estop" << std::endl;
        operator_input_triggered = true;
    }
    if (this->control.current_keyboard == Input::Keyboard::R ||
        this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        this->operator_estop_requested_.store(false, std::memory_order_relaxed);
        this->operator_rl_enable_requested_.store(false, std::memory_order_relaxed);
        this->operator_fault_reset_requested_.store(true, std::memory_order_relaxed);
        std::cout << LOGGER::INFO
                  << "[SupervisorInput] source="
                  << (this->control.current_keyboard == Input::Keyboard::R ? "keyboard:R" : "gamepad:RB_Y")
                  << " reason=fault_reset" << std::endl;
        operator_input_triggered = true;
    }
    if (this->control.current_keyboard == Input::Keyboard::Num1)
    {
        this->operator_rl_enable_requested_.store(true, std::memory_order_relaxed);
        std::cout << LOGGER::INFO
                  << "[SupervisorInput] source=keyboard:Num1 reason=rl_enable_request"
                  << std::endl;
        operator_input_triggered = true;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->control.last_keyboard = Input::Keyboard::Space;
        std::cout << LOGGER::INFO
                  << "[OperatorInput] source=keyboard:Space reason=body_velocity_zero"
                  << std::endl;
    }
    if (this->arm_split_control_enabled && this->control.current_keyboard == Input::Keyboard::Num2)
    {
        this->operator_manual_arm_request_.store(true, std::memory_order_relaxed);
        std::cout << LOGGER::INFO
                  << "[SupervisorInput] source=keyboard:Num2 reason=manual_arm_request" << std::endl;
        operator_input_triggered = true;
    }
    if (operator_input_triggered)
    {
        this->RefreshSupervisorState("operator_input");
        this->operator_fault_reset_requested_.store(false, std::memory_order_relaxed);
        this->operator_manual_arm_request_.store(false, std::memory_order_relaxed);
    }

    if (this->control.current_keyboard == Input::Keyboard::Num2)
    {
        this->HandleKey2ArmHold();
        this->control.current_keyboard = this->control.last_keyboard;
    }

    if (this->control.current_keyboard == Input::Keyboard::Num3)
    {
        this->HandleKey3ArmDefault();
        this->control.current_keyboard = this->control.last_keyboard;
    }

    if (!this->UseExclusiveRealDeployControl() && this->control.current_keyboard == Input::Keyboard::Num4)
    {
        this->HandleKey4ArmHoldToggle();
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

// - MaybePublishKey1CmdVel
// - RecordPolicyInferenceTick

// - HandleLoopException
// - LoopExceptionRequested

void RL_Real_Go2X5::RunModel()
{
    if (this->LoopExceptionRequested())
    {
        return;
    }
    if (!this->rl_init_done)
    {
        this->arm_runtime_params_ready = false;
        return;
    }
    if (!this->arm_runtime_params_ready)
    {
        Go2X5ControlLogic::ArmRuntimeStateSnapshot previous_arm_state;
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            previous_arm_state = this->CaptureArmRuntimeStateLocked();
        }
        // Re-sync arm settings after policy config (go2_x5/robot_lab/config.yaml) is loaded.
        this->InitializeArmConfig();
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            this->RestoreArmRuntimeStateLocked(previous_arm_state);
        }
        this->ValidateJointMappingOrThrow("go2_x5/robot_lab/config.yaml");
        this->SetupArmCommandSubscriber();
        this->SetupArmBridgeInterface();
        this->arm_runtime_params_ready = true;
    }

    this->episode_length_buf += 1;
    this->obs.ang_vel = this->robot_state.imu.gyroscope;
    this->obs.commands = {this->control.x, this->control.y, this->control.yaw};

#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (!this->UseExclusiveRealDeployControl() && this->control.navigation_mode)
    {
        std::lock_guard<std::mutex> lock(this->cmd_vel_mutex);
        const auto cmd = this->cmd_vel_has_filtered ? this->cmd_vel_filtered : this->cmd_vel;
#if defined(USE_ROS1) && defined(USE_ROS)
        this->obs.commands = {(float)cmd.linear.x, (float)cmd.linear.y, (float)cmd.angular.z};
#elif defined(USE_ROS2) && defined(USE_ROS)
        this->obs.commands = {(float)cmd.linear.x, (float)cmd.linear.y, (float)cmd.angular.z};
#endif
    }
#endif

    this->obs.base_quat = this->robot_state.imu.quaternion;
    this->obs.dof_pos = this->robot_state.motor_state.q;
    this->obs.dof_vel = this->robot_state.motor_state.dq;

    int arm_command_size_local = 0;
    {
        std::lock_guard<std::mutex> lock(this->arm_command_mutex);
        arm_command_size_local = this->arm_command_size;
    }

    std::vector<float> arm_hold_local;
    if (arm_command_size_local > 0)
    {
        std::vector<float> arm_obs_local;
        std::vector<float> desired_arm_command;
        {
            std::lock_guard<std::mutex> lock(this->arm_command_mutex);
            auto state = this->CaptureArmCommandStateLocked();
            desired_arm_command = state.arm_joint_command_latest;
            arm_hold_local = state.arm_hold_position;
            if (desired_arm_command.size() != static_cast<size_t>(arm_command_size_local))
            {
                desired_arm_command = arm_hold_local;
            }

            if (!desired_arm_command.empty())
            {
                arm_obs_local = Go2X5ArmRuntime::StepSmoothedCommand(&state, desired_arm_command);
                this->ApplyArmCommandStateLocked(state);
            }
        }
        if (!arm_obs_local.empty())
        {
            this->obs.arm_joint_command = arm_obs_local;
        }
    }

    this->obs.actions = this->Forward();
    this->RecordPolicyInferenceTick();
    {
        std::lock_guard<std::mutex> lock(this->policy_frame_mutex);
        this->latest_policy_command_valid_ =
            this->obs.actions.size() == static_cast<size_t>(rl_sar::protocol::kDogJointCount);
        if (this->latest_policy_command_valid_)
        {
            auto& frame = this->latest_policy_command_frame_;
            frame = rl_sar::protocol::DogPolicyCommandFrame{};
            frame.header.msg_type = rl_sar::protocol::FrameType::DogPolicyCommand;
            frame.header.seq = this->policy_seq_;
            frame.header.source_monotonic_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    this->last_policy_inference_stamp.time_since_epoch()).count());
            frame.header.publish_monotonic_ns = frame.header.source_monotonic_ns;
            frame.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
            frame.policy_id_hash = this->deploy_manifest_runtime_
                ? this->deploy_manifest_runtime_->PolicyIdHash()
                : 0ULL;
            frame.action_dim = rl_sar::protocol::kDogJointCount;
            for (size_t i = 0; i < rl_sar::protocol::kDogJointCount; ++i)
            {
                frame.leg_action[i] = this->obs.actions[i];
            }
        }
    }
    this->output_dof_pos.clear();
    this->output_dof_vel.clear();
    this->output_dof_tau.clear();

    // this->TorqueProtect(this->output_dof_tau);
    // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
    std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
    this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
}

std::vector<float> RL_Real_Go2X5::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    bool policy_health_bad = false;
    const auto observations_history = config_->GetObservationsHistory();
    try
    {
        if (!observations_history.empty())
        {
            this->history_obs_buf.insert(clamped_obs);
            this->history_obs = this->history_obs_buf.get_obs_vec(observations_history);
            actions = this->model->forward({this->history_obs});
        }
        else
        {
            actions = this->model->forward({clamped_obs});
        }
    }
    catch (const std::exception& e)
    {
        policy_health_bad = true;
        this->policy_health_bad_runtime_.store(true, std::memory_order_relaxed);
        std::cout << LOGGER::WARNING
                  << "Policy inference failed: " << e.what()
                  << ". Use zero action fallback." << std::endl;
        return this->obs.actions;
    }

    const auto action_scale = config_->GetActionScale();
    const int num_dofs = GetNumDofs();
    const size_t expected_action_dim =
        !action_scale.empty()
            ? action_scale.size()
            : static_cast<size_t>(std::max(0, num_dofs));
    const bool action_dim_mismatch =
        expected_action_dim > 0 && actions.size() != expected_action_dim;
    if (action_dim_mismatch)
    {
        std::cout << LOGGER::ERROR
                  << "Policy action dimension mismatch: expect " << expected_action_dim
                  << ", got " << static_cast<size_t>(actions.size())
                  << ". Use zero action fallback." << std::endl;
        actions.assign(expected_action_dim, 0.0f);
        policy_health_bad = true;
    }

    bool non_finite_action = false;
    for (float& value : actions)
    {
        if (!std::isfinite(value))
        {
            value = 0.0f;
            non_finite_action = true;
        }
    }
    if (non_finite_action)
    {
        policy_health_bad = true;
        std::cout << LOGGER::WARNING
                  << "Policy produced non-finite action. Replace invalid entries with zero."
                  << std::endl;
    }

    this->policy_health_bad_runtime_.store(policy_health_bad, std::memory_order_relaxed);

    const auto clip_actions_upper = config_->GetClipActionsUpper();
    const auto clip_actions_lower = config_->GetClipActionsLower();
    if (!clip_actions_upper.empty() && !clip_actions_lower.empty())
    {
        return clamp(actions, clip_actions_lower, clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_Real_Go2X5::Plot()
{
    const int num_dofs = GetNumDofs();
    const auto joint_mapping = GetJointMapping();
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    std::array<float, 20> motor_q_snapshot{};
    {
        std::lock_guard<std::mutex> lock(this->unitree_state_mutex);
        motor_q_snapshot = this->unitree_motor_q;
    }
    for (int i = 0; i < num_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        const int mapped = joint_mapping[i];
        const float real_q = (mapped >= 0 && mapped < static_cast<int>(motor_q_snapshot.size()))
            ? motor_q_snapshot[static_cast<size_t>(mapped)]
            : 0.0f;
        this->plot_real_joint_pos[i].push_back(real_q);
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(num_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

uint32_t RL_Real_Go2X5::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }

    return CRC32;
}

// Unitree communication functions moved to go2_x5_unitree_comm.cpp
// - InitLowCmd
// - QueryMotionStatus
// - QueryServiceName
// - LowStateMessageHandler

// JoystickHandler moved to go2_x5_keyboard.cpp

// ROS callbacks and data handlers moved to go2_x5_ros.cpp
// - HandleArmJointCommandData
// - HandleArmBridgeStateData
// - CmdvelCallback

volatile sig_atomic_t g_shutdown_requested_go2_x5 = 0;

void signalHandlerGo2X5(int signum)
{
    (void)signum;
    g_shutdown_requested_go2_x5 = 1;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " networkInterface" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }

    std::cout << LOGGER::INFO << "[Boot] ChannelFactory init begin, iface=" << argv[1] << std::endl;
    ChannelFactory::Instance()->Init(0, argv[1]);
    std::cout << LOGGER::INFO << "[Boot] ChannelFactory init complete" << std::endl;

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::init(argc, argv, "rl_sar_go2_x5", ros::init_options::NoSigintHandler);
    signal(SIGINT, signalHandlerGo2X5);
    RL_Real_Go2X5 rl_sar(argc, argv);
    ros::Rate rate(200.0);
    while (ros::ok() && !g_shutdown_requested_go2_x5 && !rl_sar.LoopExceptionRequested())
    {
        ros::spinOnce();
        rate.sleep();
    }
    rl_sar.SafeShutdownNow();
    if (ros::ok())
    {
        ros::shutdown();
    }
#elif defined(USE_ROS2) && defined(USE_ROS)
    signal(SIGINT, signalHandlerGo2X5);
    const auto runtime_options = ParseGo2X5RuntimeOptions(argc, argv);
    if (runtime_options.enable_ros2_runtime)
    {
        const auto ros2_argv = BuildRos2Argv(argc, argv);
        std::cout << LOGGER::INFO << "[Boot] rclcpp init begin" << std::endl;
        rclcpp::init(static_cast<int>(ros2_argv.size()), ros2_argv.data());
        std::cout << LOGGER::INFO << "[Boot] rclcpp init complete" << std::endl;
        std::cout << LOGGER::INFO << "[Boot] Constructing RL_Real_Go2X5" << std::endl;
        auto rl_sar = std::make_shared<RL_Real_Go2X5>(argc, argv);
        std::cout << LOGGER::INFO << "[Boot] RL_Real_Go2X5 constructed" << std::endl;
        rclcpp::executors::SingleThreadedExecutor executor;
        if (rl_sar->ros2_node)
        {
            executor.add_node(rl_sar->ros2_node);
        }
        while (rclcpp::ok() && !g_shutdown_requested_go2_x5 && !rl_sar->LoopExceptionRequested())
        {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        rl_sar->SafeShutdownNow();
        rclcpp::shutdown();
    }
    else
    {
        std::cout << LOGGER::INFO << "[Boot] ROS2 runtime disabled from CLI; running main control loop without rclcpp"
                  << std::endl;
        RL_Real_Go2X5 rl_sar(argc, argv);
        while (!g_shutdown_requested_go2_x5 && !rl_sar.LoopExceptionRequested())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        rl_sar.SafeShutdownNow();
    }
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    signal(SIGINT, signalHandlerGo2X5);
    RL_Real_Go2X5 rl_sar(argc, argv);
    while (!g_shutdown_requested_go2_x5 && !rl_sar.LoopExceptionRequested()) { sleep(1); }
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;
#endif

    return 0;
}

// Keyboard handler functions moved to go2_x5_keyboard.cpp
// - HandleKey2ArmHold
// - HandleKey3ArmDefault
// - HandleKey4ArmHoldToggle
