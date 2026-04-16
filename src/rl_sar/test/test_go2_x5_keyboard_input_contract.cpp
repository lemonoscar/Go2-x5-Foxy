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
    const std::filesystem::path rl_sdk_file = source_dir + "/library/core/rl_sdk/rl_sdk.cpp";
    const std::filesystem::path core_file = source_dir + "/src/rl_sar/core/rl_real_go2_x5.cpp";
    const std::filesystem::path utility_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_utility.cpp";

    Require(std::filesystem::exists(rl_sdk_file), "rl_sdk.cpp missing");
    Require(std::filesystem::exists(core_file), "rl_real_go2_x5.cpp missing");
    Require(std::filesystem::exists(utility_file), "go2_x5_utility.cpp missing");

    const std::string content = ReadAll(rl_sdk_file.string());
    RequireContains(content, "isatty(STDIN_FILENO)", rl_sdk_file.string());
    RequireContains(content, "open(\"/dev/tty\", O_RDONLY | O_NONBLOCK)", rl_sdk_file.string());
    RequireContains(
        content,
        "Keyboard input attached to /dev/tty fallback for ros2 launch.",
        rl_sdk_file.string());
    RequireContains(
        content,
        "Keyboard input unavailable: neither STDIN nor /dev/tty is interactive.",
        rl_sdk_file.string());
    RequireContains(
        content,
        "this->ConsumePendingKeyboardInput()",
        rl_sdk_file.string());
    RequireContains(
        content,
        "this->EnqueueKeyboardInput(keyboard_event);",
        rl_sdk_file.string());
    RequireContains(content, "pending_keyboard_inputs_.push_back(keyboard);", rl_sdk_file.string());
    RequireContains(content, "pending_keyboard_inputs_.pop_front();", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::Num0:", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::Num1:", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::Num2:", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::Num3:", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::Space:", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::Escape:", rl_sdk_file.string());
    RequireContains(content, "case Input::Keyboard::R:", rl_sdk_file.string());
    RequireContains(content, "this->control.current_keyboard = Input::Keyboard::None;", rl_sdk_file.string());
    RequireNotContains(content, "[OperatorInput] source=keyboard:Num0 reason=get_up_request", rl_sdk_file.string());

    const std::string core_content = ReadAll(core_file.string());
    RequireContains(core_content, "if (IsPassiveBodyOutputMode(mode))", core_file.string());
    RequireContains(core_content, "input.operator_enable = rl_active_requested;", core_file.string());
    RequireContains(core_content, "input.manual_arm_request = this->operator_manual_arm_requested_.load", core_file.string());
    RequireContains(core_content, "this->supervisor_->NoteHeartbeat(input.now_monotonic_ns);", core_file.string());
    RequireContains(core_content, "input = this->BuildSupervisorInput();", core_file.string());
    RequireContains(core_content, "[SupervisorInput] source=keyboard:Num2 reason=manual_arm_request", core_file.string());
    RequireContains(core_content, "[SupervisorInput] source=keyboard:Num3 reason=manual_arm_request", core_file.string());
    RequireNotContains(core_content, "navigation_mode", core_file.string());
    RequireNotContains(core_content, "cmd_vel_", core_file.string());
    RequireNotContains(core_content, "MaybeAdvanceKeyboardRlRequest", core_file.string());
    RequireNotContains(core_content, "CurrentFsmStateName", core_file.string());

    const std::string utility_content = ReadAll(utility_file.string());
    RequireContains(
        utility_content,
        "Control frequency: ",
        utility_file.string());
    RequireContains(
        utility_content,
        "Policy inference frequency: ",
        utility_file.string());
    RequireNotContains(utility_content, "request_get_up_then_rl", utility_file.string());
    RequireNotContains(utility_content, "request_rl_locomotion", utility_file.string());

    std::cout << "go2_x5 keyboard input contract test passed." << std::endl;
    return 0;
}
