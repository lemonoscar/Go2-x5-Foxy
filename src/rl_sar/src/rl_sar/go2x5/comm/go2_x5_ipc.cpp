
#include "rl_real_go2_x5.hpp"
#include "rl_sar/go2x5/ipc.hpp"
#include <iostream>
#include <cstring>
#include <array>

#if defined(__linux__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#endif

// ============================================================================
// IPC Communication Functions
// ============================================================================

void RL_Real_Go2X5::SetupArmCommandIpc()
{
#if defined(__linux__)
    this->CloseArmCommandIpc();
    if (this->arm_command_size <= 0 || this->arm_joint_command_port <= 0)
    {
        return;
    }

    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cout << LOGGER::WARNING << "Arm joint command IPC socket create failed: "
                  << std::strerror(errno) << std::endl;
        return;
    }

    const int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<uint16_t>(this->arm_joint_command_port));
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0)
    {
        std::cout << LOGGER::WARNING << "Arm joint command IPC bind failed on port "
                  << this->arm_joint_command_port << ": " << std::strerror(errno) << std::endl;
        close(fd);
        return;
    }

    this->arm_joint_command_socket_fd = fd;
    std::cout << LOGGER::INFO << "Arm joint command IPC ready. port="
              << this->arm_joint_command_port << std::endl;
#else
    std::cout << LOGGER::WARNING << "Arm joint command IPC is unavailable on this platform." << std::endl;
#endif
}

void RL_Real_Go2X5::CloseArmCommandIpc()
{
#if defined(__linux__)
    if (this->arm_joint_command_socket_fd >= 0)
    {
        close(this->arm_joint_command_socket_fd);
        this->arm_joint_command_socket_fd = -1;
    }
#endif
}

void RL_Real_Go2X5::PollArmCommandIpc()
{
#if defined(__linux__)
    if (this->arm_joint_command_socket_fd < 0 || this->arm_command_size <= 0)
    {
        return;
    }

    std::array<uint8_t, 1024> buffer{};
    while (true)
    {
        const ssize_t bytes_read = recv(this->arm_joint_command_socket_fd, buffer.data(), buffer.size(), 0);
        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }
            std::cout << LOGGER::WARNING << "Arm joint command IPC recv failed: "
                      << std::strerror(errno) << std::endl;
            break;
        }
        if (bytes_read == 0)
        {
            break;
        }

        std::vector<uint8_t> bytes(buffer.begin(), buffer.begin() + bytes_read);
        rl_sar::protocol::ArmCommandFrame typed_frame;
        std::string typed_error;
        if (rl_sar::protocol::ParseArmCommandFrame(bytes, typed_frame, &typed_error))
        {
            this->HandleArmJointCommandFrame(typed_frame, "Arm joint command IPC typed frame");
            continue;
        }

        Go2X5IPC::ArmPosePacket packet;
        std::string error;
        if (!Go2X5IPC::ParsePosePacket(bytes, packet, &error))
        {
            std::cout << LOGGER::WARNING << "Ignore arm joint command IPC packet: "
                      << typed_error << "; legacy_fallback=" << error << std::endl;
            continue;
        }
        if (packet.joint_count != static_cast<uint16_t>(this->arm_command_size))
        {
            std::cout << LOGGER::WARNING << "Ignore arm joint command IPC packet: expect "
                      << static_cast<int>(this->arm_command_size) << " joints, got "
                      << static_cast<int>(packet.joint_count) << std::endl;
            continue;
        }
        this->HandleArmJointCommandData(packet.q, "Arm joint command IPC");
    }
#endif
}

void RL_Real_Go2X5::SetupArmBridgeIpc()
{
#if defined(__linux__)
    this->CloseArmBridgeIpc();
    if (this->arm_joint_count <= 0)
    {
        return;
    }

    sockaddr_in cmd_addr{};
    if (!MakeIpv4Endpoint(this->arm_bridge_ipc_host, this->arm_bridge_cmd_port, cmd_addr))
    {
        std::cout << LOGGER::WARNING
                  << "Arm bridge IPC disabled: invalid IPv4 host " << this->arm_bridge_ipc_host
                  << std::endl;
        return;
    }

    const int cmd_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_fd < 0)
    {
        std::cout << LOGGER::WARNING << "Arm bridge IPC command socket create failed: "
                  << std::strerror(errno) << std::endl;
        return;
    }
    const int state_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (state_fd < 0)
    {
        std::cout << LOGGER::WARNING << "Arm bridge IPC state socket create failed: "
                  << std::strerror(errno) << std::endl;
        close(cmd_fd);
        return;
    }

    const int reuse = 1;
    setsockopt(state_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    fcntl(cmd_fd, F_SETFL, fcntl(cmd_fd, F_GETFL, 0) | O_NONBLOCK);
    fcntl(state_fd, F_SETFL, fcntl(state_fd, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<uint16_t>(this->arm_bridge_state_port));
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(state_fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0)
    {
        std::cout << LOGGER::WARNING << "Arm bridge IPC state socket bind failed on port "
                  << this->arm_bridge_state_port << ": " << std::strerror(errno) << std::endl;
        close(cmd_fd);
        close(state_fd);
        return;
    }
    if (connect(cmd_fd, reinterpret_cast<sockaddr*>(&cmd_addr), sizeof(cmd_addr)) != 0)
    {
        std::cout << LOGGER::WARNING << "Arm bridge IPC command socket connect failed to "
                  << this->arm_bridge_ipc_host << ":" << this->arm_bridge_cmd_port
                  << ": " << std::strerror(errno) << std::endl;
        close(cmd_fd);
        close(state_fd);
        return;
    }

    this->arm_bridge_cmd_socket_fd = cmd_fd;
    this->arm_bridge_state_socket_fd = state_fd;
    std::cout << LOGGER::INFO << "Arm bridge IPC ready. host=" << this->arm_bridge_ipc_host
              << ", cmd_port=" << this->arm_bridge_cmd_port
              << ", state_port=" << this->arm_bridge_state_port << std::endl;
#else
    std::cout << LOGGER::WARNING << "Arm bridge IPC is unavailable on this platform." << std::endl;
#endif
}

void RL_Real_Go2X5::CloseArmBridgeIpc()
{
#if defined(__linux__)
    if (this->arm_bridge_cmd_socket_fd >= 0)
    {
        close(this->arm_bridge_cmd_socket_fd);
        this->arm_bridge_cmd_socket_fd = -1;
    }
    if (this->arm_bridge_state_socket_fd >= 0)
    {
        close(this->arm_bridge_state_socket_fd);
        this->arm_bridge_state_socket_fd = -1;
    }
#endif
}

void RL_Real_Go2X5::PollArmBridgeIpcState()
{
#if defined(__linux__)
    if (this->arm_bridge_state_socket_fd < 0 || this->arm_joint_count <= 0)
    {
        return;
    }

    std::array<uint8_t, 2048> buffer{};
    while (true)
    {
        const ssize_t bytes_read = recv(this->arm_bridge_state_socket_fd, buffer.data(), buffer.size(), 0);
        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }
            std::cout << LOGGER::WARNING << "Arm bridge IPC state recv failed: "
                      << std::strerror(errno) << std::endl;
            break;
        }
        if (bytes_read == 0)
        {
            break;
        }

        std::vector<uint8_t> bytes(buffer.begin(), buffer.begin() + bytes_read);
        rl_sar::protocol::ArmStateFrame typed_frame;
        std::string typed_error;
        if (rl_sar::protocol::ParseArmStateFrame(bytes, typed_frame, &typed_error))
        {
            this->HandleArmBridgeStateFrame(typed_frame, "Arm bridge IPC state typed frame");
            continue;
        }

        Go2X5IPC::ArmStatePacket packet;
        std::string error;
        if (!Go2X5IPC::ParseStatePacket(bytes, packet, &error))
        {
            std::cout << LOGGER::WARNING << "Ignore arm bridge IPC state packet: "
                      << typed_error << "; legacy_fallback=" << error << std::endl;
            continue;
        }
        if (packet.joint_count != static_cast<uint16_t>(this->arm_joint_count))
        {
            std::cout << LOGGER::WARNING << "Ignore arm bridge IPC state packet: expect "
                      << static_cast<int>(this->arm_joint_count) << " joints, got "
                      << static_cast<int>(packet.joint_count) << std::endl;
            continue;
        }
        std::vector<float> data;
        data.reserve(static_cast<size_t>(this->arm_joint_count) * 3);
        data.insert(data.end(), packet.q.begin(), packet.q.end());
        data.insert(data.end(), packet.dq.begin(), packet.dq.end());
        data.insert(data.end(), packet.tau.begin(), packet.tau.end());
        this->HandleArmBridgeStateData(data, packet.state_from_backend, "Arm bridge IPC state");
    }
#endif
}

void RL_Real_Go2X5::SetupArmBridgeInterface()
{
    if (!this->arm_split_control_enabled || this->arm_joint_count <= 0)
    {
        return;
    }
    if (this->UseArmBridgeIpc())
    {
        this->SetupArmBridgeIpc();
        return;
    }

#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (this->arm_bridge_cmd_topic.empty() || this->arm_bridge_state_topic.empty())
    {
        std::cout << LOGGER::WARNING << "Arm split control enabled but bridge topics are empty." << std::endl;
        return;
    }
#if defined(USE_ROS1) && defined(USE_ROS)
    if (!this->ros1_nh)
    {
        return;
    }
    this->arm_bridge_cmd_publisher =
        this->ros1_nh->advertise<std_msgs::Float32MultiArray>(this->arm_bridge_cmd_topic, 1);
    this->arm_bridge_state_subscriber =
        this->ros1_nh->subscribe<std_msgs::Float32MultiArray>(
            this->arm_bridge_state_topic, 10, &RL_Real_Go2X5::ArmBridgeStateCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    if (!this->enable_ros2_runtime || !this->ros2_node)
    {
        std::cout << LOGGER::WARNING
                  << "Arm bridge transport is ROS, but ROS2 runtime is unavailable in rl_real_go2_x5."
                  << std::endl;
        return;
    }
    this->arm_bridge_cmd_publisher =
        this->ros2_node->create_publisher<std_msgs::msg::Float32MultiArray>(
            this->arm_bridge_cmd_topic, rclcpp::SystemDefaultsQoS());
    this->arm_bridge_state_subscriber =
        this->ros2_node->create_subscription<std_msgs::msg::Float32MultiArray>(
            this->arm_bridge_state_topic, rclcpp::SystemDefaultsQoS(),
            [this] (const std_msgs::msg::Float32MultiArray::SharedPtr msg) { this->ArmBridgeStateCallback(msg); });
#endif
    std::cout << LOGGER::INFO << "Arm bridge ready. cmd_topic=" << this->arm_bridge_cmd_topic
              << ", state_topic=" << this->arm_bridge_state_topic << std::endl;
#else
    if (this->arm_split_control_enabled)
    {
        std::cout << LOGGER::WARNING
                  << "Arm split control enabled in non-ROS build. External arm bridge is unavailable." << std::endl;
    }
#endif
}
