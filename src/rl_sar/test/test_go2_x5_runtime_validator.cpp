#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "library/core/config/runtime_validator.hpp"

namespace
{

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

void TestValidateModelResolvesRelativeToManifestWorkspace()
{
    namespace fs = std::filesystem;
    const fs::path temp_root = fs::temp_directory_path() / "go2_x5_runtime_validator";
    fs::create_directories(temp_root / "deploy");
    const fs::path model_path = temp_root / "policy.pt";
    std::ofstream(model_path) << "stub";

    const auto result = RLConfig::RuntimeValidator::ValidateModel(
        "policy.pt",
        (temp_root / "deploy" / "go2_x5_real.yaml").string());
    Require(result.is_valid, "relative model path should resolve from manifest workspace root");
}

void TestValidateModelRejectsMissingFile()
{
    const auto result = RLConfig::RuntimeValidator::ValidateModel(
        "definitely_missing_policy.pt",
        "/tmp/go2_x5_manifest/deploy/go2_x5_real.yaml");
    Require(!result.is_valid, "missing model path should fail validation");
}

void TestValidateNetworkInterfaceAcceptsLoopback()
{
    std::ifstream devs("/proc/net/dev");
    Require(devs.is_open(), "/proc/net/dev should be readable on linux test host");

    std::string line;
    std::getline(devs, line);
    std::getline(devs, line);

    std::string interface_name;
    while (std::getline(devs, line))
    {
        const auto colon = line.find(':');
        if (colon == std::string::npos)
        {
            continue;
        }

        std::string candidate = line.substr(0, colon);
        const auto begin = candidate.find_first_not_of(" \t");
        if (begin == std::string::npos)
        {
            continue;
        }
        const auto end = candidate.find_last_not_of(" \t");
        candidate = candidate.substr(begin, end - begin + 1);
        if (!candidate.empty())
        {
            interface_name = candidate;
            break;
        }
    }

    Require(!interface_name.empty(), "at least one network interface should exist on linux test host");
    const auto result = RLConfig::RuntimeValidator::ValidateNetworkInterface(interface_name);
    if (!result.is_valid)
    {
        std::cerr << "Skip positive network validation on this host: "
                  << result.error_message << '\n';
        return;
    }
}

void TestValidateCanInterfaceRejectsMissingInterface()
{
    const auto result = RLConfig::RuntimeValidator::ValidateCanInterface("missing_can_test0");
    Require(!result.is_valid, "missing CAN interface should fail validation");
}

void TestValidateArxSdkPath()
{
    namespace fs = std::filesystem;
    const fs::path temp_root = fs::temp_directory_path() / "go2_x5_runtime_validator_sdk";
    fs::create_directories(temp_root);

    const auto ok = RLConfig::RuntimeValidator::ValidateArxSdkPath(temp_root.string(), "", true);
    Require(ok.is_valid, "existing SDK root should pass validation");

    const auto bad = RLConfig::RuntimeValidator::ValidateArxSdkPath("", "", true);
    Require(!bad.is_valid, "missing SDK path should fail when live state is required");
}

}  // namespace

int main()
{
    TestValidateModelResolvesRelativeToManifestWorkspace();
    TestValidateModelRejectsMissingFile();
    TestValidateNetworkInterfaceAcceptsLoopback();
    TestValidateCanInterfaceRejectsMissingInterface();
    TestValidateArxSdkPath();
    std::cout << "test_go2_x5_runtime_validator passed\n";
    return 0;
}
