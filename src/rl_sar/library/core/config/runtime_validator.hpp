#ifndef RL_SAR_RUNTIME_VALIDATOR_HPP
#define RL_SAR_RUNTIME_VALIDATOR_HPP

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace RLConfig
{

struct RuntimeValidationResult
{
    bool is_valid = true;
    std::string error_category;
    std::string error_message;
    std::string resolved_path;

    static RuntimeValidationResult Ok()
    {
        return RuntimeValidationResult{};
    }

    static RuntimeValidationResult Error(
        const std::string& category,
        const std::string& message,
        const std::string& path = "")
    {
        RuntimeValidationResult result;
        result.is_valid = false;
        result.error_category = category;
        result.error_message = message;
        result.resolved_path = path;
        return result;
    }
};

class RuntimeValidator
{
public:
    static RuntimeValidationResult ValidateModel(const std::string& model_path,
                                                 const std::string& manifest_path)
    {
        if (model_path.empty())
        {
            return RuntimeValidationResult::Error("model", "model_path must be non-empty");
        }

        const auto resolved = ResolveCandidatePath(model_path, manifest_path);
        if (resolved.empty())
        {
            return RuntimeValidationResult::Error(
                "model",
                "model file is not reachable from runtime",
                model_path);
        }

        return RuntimeValidationResult::Ok();
    }

    static RuntimeValidationResult ValidateNetworkInterface(const std::string& interface_name)
    {
        return ValidateInterface(interface_name, "network");
    }

    static RuntimeValidationResult ValidateCanInterface(const std::string& interface_name)
    {
        return ValidateInterface(interface_name, "can");
    }

    static RuntimeValidationResult ValidateArxSdkPath(const std::string& sdk_root,
                                                      const std::string& sdk_lib_path,
                                                      bool require_live_state)
    {
        if (!require_live_state)
        {
            return RuntimeValidationResult::Ok();
        }

        namespace fs = std::filesystem;
        if (!sdk_lib_path.empty())
        {
            const fs::path lib_path(sdk_lib_path);
            if (fs::exists(lib_path))
            {
                return RuntimeValidationResult::Ok();
            }
            return RuntimeValidationResult::Error(
                "arx_sdk",
                "configured ARX SDK library path does not exist",
                sdk_lib_path);
        }

        if (!sdk_root.empty())
        {
            const fs::path root_path(sdk_root);
            if (fs::exists(root_path))
            {
                return RuntimeValidationResult::Ok();
            }
            return RuntimeValidationResult::Error(
                "arx_sdk",
                "configured ARX SDK root does not exist",
                sdk_root);
        }

        return RuntimeValidationResult::Error(
            "arx_sdk",
            "ARX SDK path is required for live backend initialization");
    }

private:
    static RuntimeValidationResult ValidateInterface(const std::string& interface_name,
                                                     const std::string& category)
    {
        if (interface_name.empty())
        {
            return RuntimeValidationResult::Error(category, "interface name must be non-empty");
        }

        int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            return RuntimeValidationResult::Error(
                category,
                std::string("failed to create ioctl socket: ") + std::strerror(errno));
        }

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

        const int rc = ::ioctl(sock, SIOCGIFINDEX, &ifr);
        const std::string error = rc == 0 ? std::string() : std::string(std::strerror(errno));
        ::close(sock);

        if (rc != 0)
        {
            return RuntimeValidationResult::Error(
                category,
                "interface is not reachable: " + error,
                interface_name);
        }

        return RuntimeValidationResult::Ok();
    }

    static std::string ResolveCandidatePath(const std::string& candidate,
                                            const std::string& manifest_path)
    {
        namespace fs = std::filesystem;
        const fs::path direct(candidate);
        if (fs::exists(direct))
        {
            return direct.string();
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
};

}  // namespace RLConfig

#endif  // RL_SAR_RUNTIME_VALIDATOR_HPP
