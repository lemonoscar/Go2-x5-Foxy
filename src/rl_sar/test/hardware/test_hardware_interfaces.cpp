/**
 * @file test_hardware_interfaces.cpp
 * @brief 硬件接口检测 - 只检测，不控制，安全
 *
 * 安全承诺：此测试不会发送任何控制命令给机器人
 * 只检测：CAN接口、网络接口、SDK库、系统资源
 */

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sys/sysinfo.h>
#include <unistd.h>

#if defined(__linux__)
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <dlfcn.h>
#endif

#include <yaml-cpp/yaml.h>

namespace Test
{

namespace fs = std::filesystem;

enum class CheckMode
{
    Offline,
    Hardware,
};

struct TestResult
{
    std::string name;
    bool passed = false;
    bool skipped = false;
    std::string message;
    double duration_ms = 0.0;
};

std::vector<TestResult> results;

class Timer
{
public:
    Timer() : start_(std::chrono::steady_clock::now()) {}

    double ElapsedMs() const
    {
        auto end = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(end - start_).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

void PrintResult(const TestResult& result)
{
    const char* status = result.skipped ? "○ SKIP" : (result.passed ? "✓ PASS" : "✗ FAIL");
    const char* color = result.skipped ? "\033[33m" : (result.passed ? "\033[32m" : "\033[31m");
    std::cout << color << status << "\033[0m "
              << result.name
              << " (" << result.duration_ms << " ms)";
    if (!result.message.empty())
    {
        std::cout << " - " << result.message;
    }
    std::cout << std::endl;
}

void PrintSummary()
{
    int passed = 0;
    int skipped = 0;
    int failed = 0;
    for (const auto& r : results)
    {
        if (r.skipped)
        {
            skipped++;
        }
        else if (r.passed)
        {
            passed++;
        }
        else
        {
            failed++;
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Test Summary: " << passed << " passed, "
              << skipped << " skipped, "
              << failed << " failed" << std::endl;
    std::cout << "========================================" << std::endl;
}

TestResult MakeSkippedResult(const std::string& name, const std::string& message)
{
    TestResult result;
    result.name = name;
    result.passed = false;
    result.skipped = true;
    result.message = message;
    return result;
}

std::string ResolveCandidatePath(const std::string& candidate, const std::string& manifest_path)
{
    if (candidate.empty())
    {
        return {};
    }

    const fs::path direct(candidate);
    if (fs::exists(direct))
    {
        return direct.lexically_normal().string();
    }

    std::vector<fs::path> roots;
    roots.push_back(fs::current_path());
    if (!manifest_path.empty())
    {
        const fs::path manifest_fs(manifest_path);
        if (manifest_fs.has_parent_path())
        {
            roots.push_back(manifest_fs.parent_path());
            if (manifest_fs.parent_path().has_parent_path())
            {
                roots.push_back(manifest_fs.parent_path().parent_path());
            }
        }
    }

    for (const auto& root : roots)
    {
        const fs::path resolved = root / candidate;
        if (fs::exists(resolved))
        {
            return resolved.lexically_normal().string();
        }
    }

    return {};
}

// ============================================================================
// Test 1: CAN接口检测（只检测，不发送）
// ============================================================================

TestResult TestCanInterface(const std::string& interface_name)
{
    TestResult result;
    result.name = "CAN Interface: " + interface_name;
    Timer timer;

    if (interface_name.empty())
    {
        result = MakeSkippedResult("CAN Interface", "Skipped (offline mode or empty interface)");
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

#if defined(__linux__)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        result.message = "Failed to create socket: " + std::string(strerror(errno));
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    int rc = ioctl(sock, SIOCGIFINDEX, &ifr);
    close(sock);

    if (rc < 0)
    {
        result.message = "Interface not found: " + std::string(strerror(errno));
    }
    else
    {
        result.passed = true;
        result.message = "Interface exists, index=" + std::to_string(ifr.ifr_ifindex) + " (检测完成，未发送数据)";
    }
#else
    result.message = "CAN test only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 2: 网络接口检测（只检测，不发送）
// ============================================================================

TestResult TestNetworkInterface(const std::string& interface_name)
{
    TestResult result;
    result.name = "Network Interface: " + interface_name;
    Timer timer;

    if (interface_name.empty())
    {
        result = MakeSkippedResult("Network Interface", "Skipped (offline mode or empty interface)");
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

#if defined(__linux__)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        result.message = "Failed to create socket: " + std::string(strerror(errno));
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    int rc = ioctl(sock, SIOCGIFINDEX, &ifr);
    if (rc >= 0)
    {
        ioctl(sock, SIOCGIFFLAGS, &ifr);
        bool is_up = (ifr.ifr_flags & IFF_UP) != 0;
        bool is_running = (ifr.ifr_flags & IFF_RUNNING) != 0;

        close(sock);

        if (is_up && is_running)
        {
            result.passed = true;
            result.message = "Interface UP and RUNNING (检测完成，未发送数据)";
        }
        else
        {
            result.message = "Interface exists but not ready (UP=" +
                           std::to_string(is_up) + " RUNNING=" + std::to_string(is_running) + ")";
        }
    }
    else
    {
        close(sock);
        result.message = "Interface not found";
    }
#else
    result.message = "Network test only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 3: Unitree SDK检测（只检测SDK文件，不初始化）
// ============================================================================

TestResult TestUnitreeSdk(const std::string& sdk_root)
{
    TestResult result;
    result.name = "Unitree SDK Files";
    Timer timer;

    std::vector<std::string> required_paths = {
        sdk_root + "/lib/libunitree_sdk2.so",
        sdk_root + "/lib/libunitree_go2.so",
        sdk_root + "/include/unitree/robot/go2/LowState_.hpp",
    };

    int missing = 0;
    for (const auto& path : required_paths)
    {
        std::ifstream f(path);
        if (!f.good())
        {
            missing++;
            if (missing <= 3)  // 只显示前3个
            {
                std::cout << "  Missing: " << path << std::endl;
            }
        }
    }

    if (missing == 0)
    {
        result.passed = true;
        result.message = "All SDK files present (检测完成，未初始化)";
    }
    else
    {
        result.message = std::to_string(missing) + " SDK files missing";
    }

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 4: ARX SDK库加载检测（只加载符号表，不执行）
// ============================================================================

TestResult TestArxSdkLibrary(const std::string& sdk_root, const std::string& sdk_lib_path)
{
    TestResult result;
    result.name = "ARX SDK Library";
    Timer timer;

#if defined(__linux__)
    std::vector<std::string> lib_paths;
    if (!sdk_lib_path.empty())
    {
        const fs::path lib_path(sdk_lib_path);
        if (fs::is_directory(lib_path))
        {
            lib_paths.push_back((lib_path / "libhardware.so").string());
        }
        else
        {
            lib_paths.push_back(lib_path.string());
        }
    }
    if (!sdk_root.empty())
    {
        lib_paths.push_back(sdk_root + "/lib/aarch64/libhardware.so");
        lib_paths.push_back(sdk_root + "/lib/libhardware.so");
    }

    std::string found_path;
    for (const auto& path : lib_paths)
    {
        std::ifstream f(path);
        if (f.good())
        {
            found_path = path;
            break;
        }
    }

    if (found_path.empty())
    {
        result.message =
            "libhardware.so not found (ARX5_SDK_ROOT=" + sdk_root + ", ARX5_SDK_LIB_PATH=" + sdk_lib_path + ")";
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    // 只加载库，不执行任何函数
    void* handle = dlopen(found_path.c_str(), RTLD_LAZY);
    if (!handle)
    {
        result.message = "dlopen failed: " + std::string(dlerror());
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    // 只检查符号存在，不调用
    const char* symbols[] = {
        "_ZN6ArxCanC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE",
        "_ZN6ArxCanD1Ev",
    };

    int missing = 0;
    for (const auto* symbol : symbols)
    {
        if (dlsym(handle, symbol) == nullptr)
        {
            missing++;
        }
    }

    dlclose(handle);

    if (missing > 0)
    {
        result.message = "Library found but " + std::to_string(missing) + " symbols missing";
    }
    else
    {
        result.passed = true;
        result.message = "Library loaded successfully (检测完成，未执行硬件操作)";
    }
#else
    result.message = "SDK loading only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 5: 系统资源检测
// ============================================================================

TestResult TestSystemResources()
{
    TestResult result;
    result.name = "System Resources";
    Timer timer;

#if defined(__linux__)
    struct sysinfo info;
    if (sysinfo(&info) != 0)
    {
        result.message = "sysinfo failed: " + std::string(strerror(errno));
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    double total_ram_gb = info.totalram * info.mem_unit / 1e9;
    double free_ram_gb = info.freeram * info.mem_unit / 1e9;
    double load_1min = info.loads[0] / (1 << SI_LOAD_SHIFT);

    std::string info_str = "RAM: " + std::to_string(static_cast<int>(free_ram_gb)) +
                          "/" + std::to_string(static_cast<int>(total_ram_gb)) + " GB, " +
                          "Load: " + std::to_string(load_1min).substr(0, 4);

    bool ram_ok = free_ram_gb > 1.0;
    bool load_ok = load_1min < 4.0;

    if (ram_ok && load_ok)
    {
        result.passed = true;
        result.message = info_str;
    }
    else
    {
        result.message = info_str + " - " +
                        (ram_ok ? "" : "LOW_RAM ") +
                        (load_ok ? "" : "HIGH_LOAD");
    }
#else
    result.message = "System resources test only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 6: Deploy Manifest检测
// ============================================================================

TestResult TestDeployManifest(const std::string& manifest_path)
{
    TestResult result;
    result.name = "Deploy Manifest";
    Timer timer;

    try
    {
        const YAML::Node manifest = YAML::LoadFile(manifest_path);
        if (!manifest || !manifest.IsMap())
        {
            result.message = "Manifest must be a YAML mapping";
            result.duration_ms = timer.ElapsedMs();
            return result;
        }

        const auto require_positive_int = [&](const char* section, const char* key) -> bool {
            const YAML::Node node = manifest[section];
            return node && node[key] && node[key].IsScalar() && node[key].as<int>() > 0;
        };
        const auto require_positive_double = [&](const char* section, const char* key) -> bool {
            const YAML::Node node = manifest[section];
            return node && node[key] && node[key].IsScalar() && node[key].as<double>() > 0.0;
        };
        const auto require_non_empty = [&](const char* section, const char* key) -> bool {
            const YAML::Node node = manifest[section];
            return node && node[key] && node[key].IsScalar() && !node[key].as<std::string>().empty();
        };

        const bool ok =
            require_positive_int("meta", "manifest_version") &&
            require_positive_int("robot", "leg_joint_count") &&
            require_positive_int("robot", "arm_joint_count") &&
            require_positive_int("policy", "action_dim") &&
            require_positive_int("policy", "observation_dim") &&
            require_positive_int("policy", "policy_rate_hz") &&
            require_positive_double("coordinator", "rate_hz") &&
            require_positive_double("supervisor", "probe_window_sec") &&
            require_non_empty("body_adapter", "network_interface") &&
            require_non_empty("arm_adapter", "can_interface") &&
            require_non_empty("policy", "model_path");

        if (ok)
        {
            result.passed = true;
            result.message = "Manifest schema and key runtime fields look valid";
        }
        else
        {
            result.message = "Missing or invalid required manifest fields";
        }
    }
    catch (const std::exception& ex)
    {
        result.message = std::string("YAML parse failed: ") + ex.what();
    }

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 7: Runtime模型可达性检测
// ============================================================================

TestResult TestRuntimeModel(const std::string& manifest_path)
{
    TestResult result;
    result.name = "Runtime Model";
    Timer timer;

    try
    {
        const YAML::Node manifest = YAML::LoadFile(manifest_path);
        if (!manifest || !manifest["policy"] || !manifest["policy"]["model_path"])
        {
            result.message = "Manifest policy.model_path missing";
            result.duration_ms = timer.ElapsedMs();
            return result;
        }

        const std::string model_path = manifest["policy"]["model_path"].as<std::string>();
        if (model_path.empty())
        {
            result.message = "Manifest policy.model_path is empty";
            result.duration_ms = timer.ElapsedMs();
            return result;
        }

        const std::string resolved = ResolveCandidatePath(model_path, manifest_path);
        if (resolved.empty())
        {
            result.message = "Model file not reachable from manifest: " + model_path;
            result.duration_ms = timer.ElapsedMs();
            return result;
        }

        const auto file_size = fs::file_size(fs::path(resolved));
        result.passed = true;
        result.message =
            "Resolved: " + resolved + " (" + std::to_string(static_cast<unsigned long long>(file_size / 1024ULL)) +
            " KB)";
    }
    catch (const std::exception& ex)
    {
        result.message = std::string("Runtime model validation failed: ") + ex.what();
    }

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 8: 策略目录检测（兼容旧脚本）
// ============================================================================

TestResult TestPolicyDirectory(const std::string& policy_dir)
{
    TestResult result;
    result.name = "Policy Directory";
    Timer timer;

    if (policy_dir.empty())
    {
        result = MakeSkippedResult("Policy Directory", "Skipped (empty policy_dir)");
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    const std::vector<std::string> policy_files = {
        policy_dir + "/policy.pt",
        policy_dir + "/policy.onnx",
    };

    for (const auto& file : policy_files)
    {
        std::ifstream f(file);
        if (f.good())
        {
            f.seekg(0, std::ios::end);
            const auto size = static_cast<unsigned long long>(f.tellg());
            f.close();

            result.passed = true;
            result.message = "Found: " + file + " (" + std::to_string(size / 1024ULL) + " KB)";
            result.duration_ms = timer.ElapsedMs();
            return result;
        }
    }

    result.message = "No policy file found in " + policy_dir;
    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// 安全声明
// ============================================================================

void PrintSafetyNotice()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  ⚠️  安 全 提 示" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "此测试套件只会检测硬件接口和配置文件。" << std::endl;
    std::cout << "不会发送任何控制命令给机器人。" << std::endl;
    std::cout << "机器人保持静止状态。" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

} // namespace Test

int main(int argc, char** argv)
{
    Test::PrintSafetyNotice();

    std::cout << "========================================" << std::endl;
    std::cout << "  Go2-X5 启动前检查 (只检测，不控制)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // 默认配置
    Test::CheckMode mode = Test::CheckMode::Offline;
    std::string can_interface = "can0";
    std::string network_interface = "eth0";
    std::string arx_sdk_root = std::getenv("ARX5_SDK_ROOT") ? std::getenv("ARX5_SDK_ROOT") : "";
    std::string arx_sdk_lib_path = std::getenv("ARX5_SDK_LIB_PATH") ? std::getenv("ARX5_SDK_LIB_PATH") : "";
    std::string unitree_sdk_root = std::getenv("UNITREE_SDK2_ROOT") ? std::getenv("UNITREE_SDK2_ROOT") : "";
    std::string manifest_path = "deploy/go2_x5_real.yaml";
    std::string policy_dir = ".";

    // 解析命令行参数
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--can" && i + 1 < argc)
        {
            can_interface = argv[++i];
        }
        else if (arg == "--net" && i + 1 < argc)
        {
            network_interface = argv[++i];
        }
        else if (arg == "--arx-sdk" && i + 1 < argc)
        {
            arx_sdk_root = argv[++i];
        }
        else if (arg == "--arx-sdk-lib" && i + 1 < argc)
        {
            arx_sdk_lib_path = argv[++i];
        }
        else if (arg == "--unitree-sdk" && i + 1 < argc)
        {
            unitree_sdk_root = argv[++i];
        }
        else if (arg == "--manifest" && i + 1 < argc)
        {
            manifest_path = argv[++i];
        }
        else if (arg == "--policy-dir" && i + 1 < argc)
        {
            policy_dir = argv[++i];
        }
        else if (arg == "--mode" && i + 1 < argc)
        {
            const std::string mode_text = argv[++i];
            if (mode_text == "hardware")
            {
                mode = Test::CheckMode::Hardware;
            }
            else if (mode_text == "offline")
            {
                mode = Test::CheckMode::Offline;
            }
            else
            {
                std::cerr << "Unknown --mode value: " << mode_text << std::endl;
                return 2;
            }
        }
        else if (arg == "--help")
        {
            std::cout << "用法: " << argv[0] << " [选项]\n"
                      << "\n选项:\n"
                      << "  --mode <offline|hardware> 启动前检查模式 (默认: offline)\n"
                      << "  --can <接口>        CAN接口名 (默认: can0)\n"
                      << "  --net <接口>        网络接口名 (默认: eth0)\n"
                      << "  --arx-sdk <路径>    ARX SDK路径 (默认: $ARX5_SDK_ROOT)\n"
                      << "  --arx-sdk-lib <路径> ARX SDK库目录或libhardware.so路径\n"
                      << "  --unitree-sdk <路径> Unitree SDK路径 (默认: $UNITREE_SDK2_ROOT)\n"
                      << "  --manifest <路径>   Deploy manifest路径\n"
                      << "  --policy-dir <路径>  策略文件目录\n"
                      << "  --help               显示此帮助\n"
                      << std::endl;
            return 0;
        }
    }

    // 运行测试
    if (mode == Test::CheckMode::Hardware)
    {
        Test::results.push_back(Test::TestCanInterface(can_interface));
        Test::results.push_back(Test::TestNetworkInterface(network_interface));
    }
    else
    {
        Test::results.push_back(Test::MakeSkippedResult("CAN Interface", "Skipped in offline mode"));
        Test::results.push_back(Test::MakeSkippedResult("Network Interface", "Skipped in offline mode"));
    }

    if (!unitree_sdk_root.empty())
    {
        Test::results.push_back(Test::TestUnitreeSdk(unitree_sdk_root));
    }
    else
    {
        Test::TestResult r;
        r.name = "Unitree SDK Files";
        r.skipped = true;
        r.message = "Skipped (UNITREE_SDK2_ROOT not set)";
        Test::results.push_back(r);
    }

    if (!arx_sdk_root.empty() || !arx_sdk_lib_path.empty())
    {
        Test::results.push_back(Test::TestArxSdkLibrary(arx_sdk_root, arx_sdk_lib_path));
    }
    else
    {
        Test::TestResult r;
        r.name = "ARX SDK Library";
        r.skipped = true;
        r.message = "Skipped (ARX5_SDK_ROOT / ARX5_SDK_LIB_PATH not set)";
        Test::results.push_back(r);
    }

    Test::results.push_back(Test::TestSystemResources());
    Test::results.push_back(Test::TestDeployManifest(manifest_path));
    Test::results.push_back(Test::TestRuntimeModel(manifest_path));
    Test::results.push_back(Test::TestPolicyDirectory(policy_dir));

    // 打印结果
    std::cout << std::endl;
    for (const auto& r : Test::results)
    {
        Test::PrintResult(r);
    }

    Test::PrintSummary();

    // 最终安全提示
    std::cout << "\n⚠️  测试完成。所有检测均为只读，机器人未受影响。" << std::endl;

    int failed = 0;
    for (const auto& r : Test::results)
    {
        if (!r.passed && !r.skipped) failed++;
    }

    return (failed == 0) ? 0 : 1;
}
