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

} // namespace

int main()
{
    const std::string source_dir = RL_SAR_SOURCE_DIR;
    const std::filesystem::path rl_sdk_file = source_dir + "/library/core/rl_sdk/rl_sdk.cpp";

    Require(std::filesystem::exists(rl_sdk_file), "rl_sdk.cpp missing");

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

    std::cout << "go2_x5 keyboard input contract test passed." << std::endl;
    return 0;
}
