/**
 * @file test_hardware_interfaces.cpp
 * @brief 硬件接口测试 - 不需要真实机器人，只测试接口可用性
 *
 * 测试内容：
 * 1. CAN接口检测
 * 2. 网络接口检测
 * 3. ARX SDK库加载
 * 4. Unitree SDK库加载
 * 5. 内存和CPU资源检测
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
// Test 1: CAN接口检测
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
        result.message = "Interface exists, index=" + std::to_string(ifr.ifr_ifindex);
    }
#else
    result.message = "CAN test only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 2: 网络接口检测
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
        // 检查接口是否UP
        ioctl(sock, SIOCGIFFLAGS, &ifr);
        bool is_up = (ifr.ifr_flags & IFF_UP) != 0;
        bool is_running = (ifr.ifr_flags & IFF_RUNNING) != 0;

        close(sock);

        if (is_up && is_running)
        {
            result.passed = true;
            result.message = "Interface UP and RUNNING";
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
// Test 3: ARX SDK库加载
// ============================================================================

TestResult TestArxSdkLibrary(const std::string& sdk_root)
{
    TestResult result;
    result.name = "ARX SDK Library";
    Timer timer;

#if defined(__linux__)
    // 尝试多个可能的路径
    std::vector<std::string> lib_paths = {
        sdk_root + "/lib/aarch64/libhardware.so",
        sdk_root + "/lib/libhardware.so",
        "/usr/local/lib/libhardware.so",
    };

    bool found = false;
    std::string found_path;

    for (const auto& path : lib_paths)
    {
        std::ifstream f(path);
        if (f.good())
        {
            found = true;
            found_path = path;
            break;
        }
    }

    if (!found)
    {
        result.message = "libhardware.so not found (searched: ARX5_SDK_ROOT=" + sdk_root + ")";
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    // 尝试加载库
    void* handle = dlopen(found_path.c_str(), RTLD_LAZY);
    if (!handle)
    {
        result.message = "dlopen failed: " + std::string(dlerror());
        result.duration_ms = timer.ElapsedMs();
        return result;
    }

    // 检查关键符号
    const char* symbols[] = {
        "_ZN6ArxCanC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE",
        "_ZN6ArxCanD1Ev",
        "_ZN6ArxCan17send_EC_motor_cmdEtfffff",
        "_ZN6ArxCan17send_DM_motor_cmdEtfffff",
        "_ZN6ArxCan13get_motor_msgEv",
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
        result.message = "Found library but " + std::to_string(missing) + " symbols missing";
    }
    else
    {
        result.passed = true;
        result.message = "Library loaded: " + found_path;
    }
#else
    result.message = "SDK loading only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 4: 系统资源检测
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

    // 总内存 (GB)
    double total_ram_gb = info.totalram * info.mem_unit / 1e9;
    // 空闲内存 (GB)
    double free_ram_gb = info.freeram * info.mem_unit / 1e9;
    // 1分钟负载平均值
    double load_1min = info.loads[0] / (1 << SI_LOAD_SHIFT);

    std::string info_str = "RAM: " + std::to_string(static_cast<int>(free_ram_gb)) +
                          "/" + std::to_string(static_cast<int>(total_ram_gb)) + " GB, " +
                          "Load: " + std::to_string(load_1min).substr(0, 4);

    // 检查最低要求
    bool ram_ok = free_ram_gb > 1.0;  // 至少1GB空闲
    bool load_ok = load_1min < 4.0;   // 负载不过高

    if (ram_ok && load_ok)
    {
        result.passed = true;
        result.message = info_str;
    }
    else
    {
        result.message = info_str + " - " +
                        (ram_ok ? "" : "LOW RAM ") +
                        (load_ok ? "" : "HIGH LOAD");
    }
#else
    result.message = "System resources test only supported on Linux";
#endif

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 5: CPU性能测试
// ============================================================================

TestResult TestCpuPerformance()
{
    TestResult result;
    result.name = "CPU Performance (quick check)";
    Timer timer;

    // 执行一些浮点运算
    const int iterations = 1000000;
    volatile double sum = 0.0;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
        sum += std::sin(i * 0.001) * std::cos(i * 0.001);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    double ops_per_sec = (iterations * 2) / (duration_ms / 1000.0);  // 每秒运算数

    // Jetson NX应该能达到至少100 MFLOPS
    result.passed = (ops_per_sec > 1e8);  // 100 MFLOPS
    result.message = "Performance: " + std::to_string(static_cast<int>(ops_per_sec / 1e6)) +
                    " MFLOPS, duration: " + std::to_string(duration_ms) + " ms";

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Test 6: 磁盘写入测试
// ============================================================================

TestResult TestDiskWrite(const std::string& test_dir)
{
    TestResult result;
    result.name = "Disk Write Test";
    Timer timer;

    std::string test_file = test_dir + "/write_test.tmp";
    std::vector<uint8_t> data(1024 * 1024);  // 1MB

    auto start = std::chrono::high_resolution_clock::now();

    {
        std::ofstream out(test_file, std::ios::binary);
        if (!out)
        {
            result.message = "Failed to create test file in " + test_dir;
            result.duration_ms = timer.ElapsedMs();
            return result;
        }
        out.write(reinterpret_cast<const char*>(data.data()), data.size());
    }

    auto end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    double mb_per_sec = (1.0 / (duration_ms / 1000.0));

    // 清理
    std::remove(test_file.c_str());

    result.passed = (mb_per_sec > 10.0);  // 至少10 MB/s
    result.message = "Write speed: " + std::to_string(static_cast<int>(mb_per_sec)) + " MB/s";

    result.duration_ms = timer.ElapsedMs();
    return result;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Go2-X5 Hardware Interface Test" << std::endl;
    std::cout << "  Running on: " << sysconf(_SC_MACHINE) << " cores" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // 默认配置
    std::string can_interface = "can0";
    std::string network_interface = "eth0";
    std::string arx_sdk_root = std::getenv("ARX5_SDK_ROOT") ? std::getenv("ARX5_SDK_ROOT") : "";
    std::string test_dir = "/tmp";

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
        else if (arg == "--test-dir" && i + 1 < argc)
        {
            test_dir = argv[++i];
        }
    }

    // 运行测试
    results.push_back(TestCanInterface(can_interface));
    results.push_back(TestNetworkInterface(network_interface));

    if (!arx_sdk_root.empty())
    {
        results.push_back(TestArxSdkLibrary(arx_sdk_root));
    }
    else
    {
        TestResult r;
        r.name = "ARX SDK Library";
        r.message = "Skipped (ARX5_SDK_ROOT not set)";
        results.push_back(r);
    }

    results.push_back(TestSystemResources());
    results.push_back(TestCpuPerformance());
    results.push_back(TestDiskWrite(test_dir));

    // 打印结果
    std::cout << std::endl;
    for (const auto& r : results)
    {
        PrintResult(r);
    }

    PrintSummary();

    // 返回值
    int failed = 0;
    for (const auto& r : results)
    {
        if (!r.passed) failed++;
    }

    return (failed == 0) ? 0 : 1;
}

} // namespace Test

int main(int argc, char** argv)
{
    return Test::main(argc, argv);
}
