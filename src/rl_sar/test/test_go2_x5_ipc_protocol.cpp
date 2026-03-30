#include <iostream>
#include <string>
#include <vector>

#include "rl_sar/go2x5/comm/go2_x5_ipc_protocol.hpp"

int main()
{
    using namespace Go2X5IPC;

    {
        const std::vector<float> q = {0.0f, 1.0f, 2.0f};
        const std::vector<float> dq = {0.1f, 0.2f, 0.3f};
        const std::vector<float> kp = {10.0f, 11.0f, 12.0f};
        const std::vector<float> kd = {1.0f, 1.1f, 1.2f};
        const std::vector<float> tau = {0.5f, 0.6f, 0.7f};
        const auto bytes = SerializeCommandPacket(3, q, dq, kp, kd, tau);
        if (bytes.size() != CommandPacketSize(3))
        {
            std::cerr << "Unexpected serialized command packet size\n";
            return 1;
        }

        ArmCommandPacket packet;
        std::string error;
        if (!ParseCommandPacket(bytes, packet, &error))
        {
            std::cerr << "Failed to parse command packet: " << error << "\n";
            return 1;
        }
        if (packet.joint_count != 3 || packet.q != q || packet.dq != dq || packet.kp != kp || packet.kd != kd || packet.tau != tau)
        {
            std::cerr << "Command packet roundtrip mismatch\n";
            return 1;
        }
    }

    {
        const std::vector<float> q = {0.3f, 0.4f};
        const std::vector<float> dq = {0.0f, -0.1f};
        const std::vector<float> tau = {0.7f, 0.8f};
        const auto bytes = SerializeStatePacket(2, true, q, dq, tau);
        if (bytes.size() != StatePacketSize(2))
        {
            std::cerr << "Unexpected serialized state packet size\n";
            return 1;
        }

        ArmStatePacket packet;
        std::string error;
        if (!ParseStatePacket(bytes, packet, &error))
        {
            std::cerr << "Failed to parse state packet: " << error << "\n";
            return 1;
        }
        if (packet.joint_count != 2 || !packet.state_from_backend || packet.q != q || packet.dq != dq || packet.tau != tau)
        {
            std::cerr << "State packet roundtrip mismatch\n";
            return 1;
        }
    }

    {
        const std::vector<float> q = {0.6f, 1.2f, 1.8f, 2.4f};
        const auto bytes = SerializePosePacket(4, q);
        if (bytes.size() != PosePacketSize(4))
        {
            std::cerr << "Unexpected serialized pose packet size\n";
            return 1;
        }

        ArmPosePacket packet;
        std::string error;
        if (!ParsePosePacket(bytes, packet, &error))
        {
            std::cerr << "Failed to parse pose packet: " << error << "\n";
            return 1;
        }
        if (packet.joint_count != 4 || packet.q != q)
        {
            std::cerr << "Pose packet roundtrip mismatch\n";
            return 1;
        }
    }

    {
        std::vector<uint8_t> invalid = {0, 1, 2, 3, 4};
        ArmCommandPacket packet;
        if (ParseCommandPacket(invalid, packet))
        {
            std::cerr << "Expected invalid command packet to be rejected\n";
            return 1;
        }
    }

    {
        const auto bytes = SerializeStatePacket(1, false, {1.0f}, {0.0f}, {0.0f});
        std::vector<uint8_t> invalid = bytes;
        invalid[3] = 'X';
        ArmStatePacket packet;
        if (ParseStatePacket(invalid, packet))
        {
            std::cerr << "Expected invalid state packet magic to be rejected\n";
            return 1;
        }
    }

    {
        const auto bytes = SerializePosePacket(1, {1.0f});
        std::vector<uint8_t> invalid = bytes;
        invalid[3] = 'X';
        ArmPosePacket packet;
        if (ParsePosePacket(invalid, packet))
        {
            std::cerr << "Expected invalid pose packet magic to be rejected\n";
            return 1;
        }
    }

    {
        if (!IsIpcTransport("ipc") || !IsIpcTransport("IPC") || IsIpcTransport("ros"))
        {
            std::cerr << "Transport normalization is incorrect\n";
            return 1;
        }
    }

    std::cout << "test_go2_x5_ipc_protocol passed\n";
    return 0;
}
