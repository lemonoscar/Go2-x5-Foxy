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
    const std::filesystem::path config_file =
        source_dir + "/library/core/config/go2_x5_layered_config.hpp";

    const std::string content = ReadAll(config_file.string());
    RequireContains(content, "const int leg_action_dim = std::max(0, num_of_dofs - arm_joint_count);", config_file.string());
    RequireContains(content, "\"action_scale\",", config_file.string());
    RequireContains(content, "static_cast<size_t>(leg_action_dim),", config_file.string());

    std::cout << "test_go2_x5_dog_only_config_contract passed\n";
    return 0;
}
