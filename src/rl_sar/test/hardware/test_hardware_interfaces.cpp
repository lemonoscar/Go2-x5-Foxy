/**
 * @file test_hardware_interfaces.cpp
 * @brief 硬件接口检测 - 只检测，不控制，安全
 *
 * 安全承诺：此测试不会发送任何控制命令给机器人
 * 只检测：CAN接口、网络接口、SDK库、系统资源
 */

#include <chrono>
#include <cstring>
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

namespace Test
{

struct TestResult
{
    std::string name;
    bool passed = false;
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
    const char* status = result.passed ? "✓ PASS" : "✗ FAIL";
    const char* color = result.passed ? "\033[32m" : "\033[31m";
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
    int failed = 0;
    for (const auto& r : results)
    {
        if (r.passed) passed++;
        else failed++;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Test Summary: " << passed << " passed, " << failed << " failed" << std::endl;
    std::cout << "========================================" << std::endl;
}

// ============================================================================
// Test 1: CAN接口检测（只检测，不发送）
// ============================================================================

TestResult TestCanInterface(const std::string& interface_name)
{
    TestResult result;
    result.name = "CAN Interface: " + interface_name;
    Timer timer;

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

TestResult TestArxSdkLibrary(const std::string& sdk_root)
{
    TestResult result;
    result.name = "ARX SDK Library";
    Timer timer;

#if defined(__linux__)
    std::vector<std::string> lib_paths = {
        sdk_root + "/lib/aarch64/libhardware.so",
        sdk_root + "/lib/libhardware.so",
    };

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
        result.message = "libhardware.so not found (ARX5_SDK_ROOT=" + sdk_root + ")";
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

    std::ifstream f(manifest_path);
    if (!f)
    {
        result.message = "File not found: " + manifest_path;
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    // 检查关键配置项
    std::vector<std::string> required_keys = {
        "meta.manifest_version",
        "robot.leg_joint_count",
        "robot.arm_joint_count",
        "policy.action_dim",
        "policy.observation_dim",
        "coordinator.rate_hz",
    };

    std::string line;
    int found_keys = 0;
    while (std::getline(f, line))
    {
        for (const auto& key : required_keys)
        {
            if (line.find(key) != std::string::npos)
            {
                found_keys++;
                break;
            }
        }
    }

    if (found_keys >= required_keys.size())
    {
        result.passed = true;
        result.message = "Manifest OK, " + std::to_string(found_keys) + " keys verified";
    }
    else
    {
        result.message = "Missing " + std::to_string(required_keys.size() - found_keys) + " required keys";
    }

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 7: 策略文件检测
// ============================================================================

TestResult TestPolicyFile(const std::string& policy_dir)
{
    TestResult result;
    result.name = "Policy File";
    Timer timer;

    std::vector<std::string> policy_files = {
        policy_dir + "/policy.pt",
        policy_dir + "/policy.onnx",
    };

    for (const auto& file : policy_files)
    {
        std::ifstream f(file);
        if (f.good())
        {
            f.seekg(0, std::ios::end);
            size_t size = f.tellg();
            f.close();

            result.passed = true;
            result.message = "Found: " + file + " (" + std::to_string(size / 1024) + " KB)";
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
    std::cout << "  Go2-X5 硬件接口检测 (只检测，不控制)" << std::endl;
    std::cout << "  运行平台: Jetson (机器人身上)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // 默认配置
    std::string can_interface = "can0";
    std::string network_interface = "eth0";
    std::string arx_sdk_root = std::getenv("ARX5_SDK_ROOT") ? std::getenv("ARX5_SDK_ROOT") : "";
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
        else if (arg == "--help")
        {
            std::cout << "用法: " << argv[0] << " [选项]\n"
                      << "\n选项:\n"
                      << "  --can <接口>        CAN接口名 (默认: can0)\n"
                      << "  --net <接口>        网络接口名 (默认: eth0)\n"
                      << "  --arx-sdk <路径>    ARX SDK路径 (默认: $ARX5_SDK_ROOT)\n"
                      << "  --unitree-sdk <路径> Unitree SDK路径 (默认: $UNITREE_SDK2_ROOT)\n"
                      << "  --manifest <路径>   Deploy manifest路径\n"
                      << "  --policy-dir <路径>  策略文件目录\n"
                      << "  --help               显示此帮助\n"
                      << std::endl;
            return 0;
        }
    }

    // 运行测试
    results.push_back(Test::TestCanInterface(can_interface));
    results.push_back(Test::TestNetworkInterface(network_interface));

    if (!unitree_sdk_root.empty())
    {
        results.push_back(Test::TestUnitreeSdk(unitree_sdk_root));
    }
    else
    {
        TestResult r;
        r.name = "Unitree SDK Files";
        r.message = "Skipped (UNITREE_SDK2_ROOT not set)";
        results.push_back(r);
    }

    if (!arx_sdk_root.empty())
    {
        results.push_back(Test::TestArxSdkLibrary(arx_sdk_root));
    }
    else
    {
        TestResult r;
        r.name = "ARX SDK Library";
        r.message = "Skipped (ARX5_SDK_ROOT not set)";
        results.push_back(r);
    }

    results.push_back(Test::TestSystemResources());
    results.push_back(Test::TestDeployManifest(manifest_path));
    results.push_back(Test::TestPolicyFile(policy_dir));

    // 打印结果
    std::cout << std::endl;
    for (const auto& r : results)
    {
        Test::PrintResult(r);
    }

    Test::PrintSummary();

    // 最终安全提示
    std::cout << "\n⚠️  测试完成。所有检测均为只读，机器人未受影响。" << std::endl;

    int failed = 0;
    for (const auto& r : results)
    {
        if (!r.passed) failed++;
    }

    return (failed == 0) ? 0 : 1;
}
