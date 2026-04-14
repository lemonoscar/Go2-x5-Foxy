#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace
{

std::string ReadAll(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open: " << path << std::endl;
        std::abort();
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << std::endl;
        std::abort();
    }
}

void RequireContains(const std::string& content, const std::string& needle, const std::string& file)
{
    if (content.find(needle) == std::string::npos)
    {
        std::cerr << "Missing expected snippet in " << file << ": " << needle << std::endl;
        std::abort();
    }
}

void RequireNotContains(const std::string& content, const std::string& needle, const std::string& file)
{
    if (content.find(needle) != std::string::npos)
    {
        std::cerr << "Unexpected snippet in " << file << ": " << needle << std::endl;
        std::abort();
    }
}

} // namespace

int main()
{
    const std::string source_dir = RL_SAR_SOURCE_DIR;
    const std::filesystem::path fsm_file = source_dir + "/fsm_robot/fsm_go2_x5.hpp";
    const std::filesystem::path core_file = source_dir + "/src/rl_sar/core/rl_real_go2_x5.cpp";
    const std::filesystem::path utility_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_utility.cpp";
    const std::filesystem::path state_io_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_state_io.cpp";
    const std::filesystem::path ros_file = source_dir + "/src/rl_sar/go2x5/comm/go2_x5_ros.cpp";
    const std::filesystem::path ipc_file = source_dir + "/src/rl_sar/go2x5/comm/go2_x5_ipc.cpp";
    const std::filesystem::path adapter_file = source_dir + "/src/rl_sar/adapters/arx/arx_adapter.cpp";
    const std::filesystem::path shutdown_file = source_dir + "/src/rl_sar/go2x5/safety/go2_x5_safe_shutdown.cpp";
    const std::filesystem::path bridge_file = source_dir + "/scripts/arx_x5_bridge.py";

    Require(std::filesystem::exists(fsm_file), "fsm_go2_x5.hpp missing");
    Require(std::filesystem::exists(core_file), "rl_real_go2_x5.cpp missing");
    Require(std::filesystem::exists(utility_file), "go2_x5_utility.cpp missing");
    Require(std::filesystem::exists(state_io_file), "go2_x5_state_io.cpp missing");
    Require(std::filesystem::exists(ros_file), "go2_x5_ros.cpp missing");
    Require(std::filesystem::exists(ipc_file), "go2_x5_ipc.cpp missing");
    Require(std::filesystem::exists(adapter_file), "arx_adapter.cpp missing");
    Require(std::filesystem::exists(shutdown_file), "go2_x5_safe_shutdown.cpp missing");
    Require(!std::filesystem::exists(bridge_file), "arx_x5_bridge.py should be removed");

    const std::string fsm_content = ReadAll(fsm_file.string());
    RequireContains(fsm_content, "fsm_command->motor_command.q[i] = kPassivePosStopF;", fsm_file.string());
    RequireContains(fsm_content, "fsm_command->motor_command.dq[i] = kPassiveVelStopF;", fsm_file.string());
    RequireContains(fsm_content, "fsm_command->motor_command.kp[i] = 0.0f;", fsm_file.string());
    RequireContains(fsm_content, "fsm_command->motor_command.kd[i] = 0.0f;", fsm_file.string());

    const std::string core_content = ReadAll(core_file.string());
    RequireContains(
        core_content,
        "if (IsPassiveBodyJointCommand(old_q, old_dq, old_kp, old_kd, old_tau))",
        core_file.string());
    RequireContains(
        core_content,
        "auto build_body_command_from_robot_command =",
        core_file.string());
    RequireContains(
        core_content,
        "command_frame.q[body_idx] = command_local.motor_command.q[joint_slot];",
        core_file.string());
    RequireContains(
        core_content,
        "final_body_command = build_body_command_from_robot_command(supervisor_mode);",
        core_file.string());
    RequireContains(
        core_content,
        "[Boot] Arm actuation unavailable: ArxAdapter inactive. ",
        core_file.string());
    RequireNotContains(
        core_content,
        "final_body_command = this->BuildHoldBodyCommandFrame(now_monotonic_ns, supervisor_mode);",
        core_file.string());
    RequireNotContains(
        core_content,
        "[Boot] Setting up external arm bridge interface",
        core_file.string());
    RequireNotContains(
        core_content,
        "this->SetupArmBridgeInterface();",
        core_file.string());

    const std::string utility_content = ReadAll(utility_file.string());
    RequireContains(utility_content, "bool RL_Real_Go2X5::ShouldActuateArmForMode", utility_file.string());
    RequireContains(utility_content, "case Go2X5Supervisor::Mode::ManualArm:", utility_file.string());
    RequireContains(utility_content, "case Go2X5Supervisor::Mode::RlDogOnlyActive:", utility_file.string());
    RequireContains(utility_content, "bool RL_Real_Go2X5::ShouldExecuteActiveShutdown()", utility_file.string());

    const std::string state_io_content = ReadAll(state_io_file.string());
    RequireContains(state_io_content, "const auto supervisor_mode = this->GetSupervisorModeSnapshot();", state_io_file.string());
    RequireContains(state_io_content, "const bool allow_arm_actuation =", state_io_file.string());
    RequireContains(
        state_io_content,
        "this->arm_safe_shutdown_active.load() || this->ShouldActuateArmForMode(supervisor_mode);",
        state_io_file.string());
    RequireContains(state_io_content, "Arm command suppressed in supervisor mode", state_io_file.string());
    RequireContains(state_io_content, "Waiting for Ready, ManualArm, or RlDogOnlyActive.", state_io_file.string());
    RequireNotContains(
        state_io_content,
        "const bool allow_passthrough =",
        state_io_file.string());

    const std::string ros_content = ReadAll(ros_file.string());
    RequireContains(ros_content, "const bool monitor_tracking_error = this->ShouldActuateArmForMode(supervisor_mode);", ros_file.string());
    RequireContains(ros_content, "this->arm_tracking_error_high_stamp = std::chrono::steady_clock::time_point{};", ros_file.string());

    const std::string ipc_content = ReadAll(ipc_file.string());
    RequireContains(
        ipc_content,
        "Arm bridge fallback has been removed. ",
        ipc_file.string());

    const std::string adapter_content = ReadAll(adapter_file.string());
    RequireContains(
        adapter_content,
        "BridgeBackend removed - only InProcessSdk is supported",
        adapter_file.string());
    RequireContains(
        adapter_content,
        "return \"sdk_inprocess_arxcan\";",
        adapter_file.string());

    const std::string shutdown_content = ReadAll(shutdown_file.string());
    RequireContains(shutdown_content, "if (this->arm_split_control_enabled && this->IsArmJointIndex(i))", shutdown_file.string());
    RequireContains(shutdown_content, "const auto default_pos = GetDefaultDofPos();", shutdown_file.string());
    RequireContains(shutdown_content, "if (this->robot_state.motor_state.q.size() >= static_cast<size_t>(num_dofs))", shutdown_file.string());
    RequireContains(shutdown_content, "if (this->ShouldExecuteActiveShutdown())", shutdown_file.string());
    RequireContains(shutdown_content, "[Shutdown] Passive shutdown: body command already inactive, skip soft land.", shutdown_file.string());

    std::cout << "test_go2_x5_startup_safety_contract passed\n";
    return 0;
}
