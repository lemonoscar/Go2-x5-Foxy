#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include "rl_sar/go2x5/arm/go2_x5_arm_transport.hpp"
#include "rl_sar/go2x5/comm/go2_x5_ipc_protocol.hpp"

using namespace Go2X5ArmTransport;
using namespace Go2X5IPC;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << "\n"; \
            return 1; \
        } \
    } while(0)

#define TEST_FLOAT_EQ(a, b, msg) \
    TEST_CHECK(std::fabs((a) - (b)) < 1e-6f, msg)

int main() {
    int test_count = 0;
    int passed = 0;

    // Test 1: ArmCommandIsValid
    {
        test_count++;
        ArmCommand cmd;
        cmd.q = {0.0f, 1.0f, 2.0f};
        cmd.dq = {0.1f, 0.1f, 0.1f};
        cmd.kp = {10.0f, 10.0f, 10.0f};
        cmd.kd = {1.0f, 1.0f, 1.0f};
        cmd.tau = {0.0f, 0.0f, 0.0f};

        if (cmd.is_valid() && cmd.joint_count() == 3u) {
            passed++;
            std::cout << "✓ ArmCommandIsValid\n";
        } else {
            std::cerr << "✗ ArmCommandIsValid\n";
        }
    }

    // Test 2: ArmCommandInvalidWhenSizesMismatch
    {
        test_count++;
        ArmCommand cmd;
        cmd.q = {0.0f, 1.0f, 2.0f};
        cmd.dq = {0.1f, 0.1f};

        if (!cmd.is_valid()) {
            passed++;
            std::cout << "✓ ArmCommandInvalidWhenSizesMismatch\n";
        } else {
            std::cerr << "✗ ArmCommandInvalidWhenSizesMismatch\n";
        }
    }

    // Test 3: ArmCommandEmptyIsValid
    {
        test_count++;
        ArmCommand cmd;

        if (cmd.is_valid() && cmd.joint_count() == 0u) {
            passed++;
            std::cout << "✓ ArmCommandEmptyIsValid\n";
        } else {
            std::cerr << "✗ ArmCommandEmptyIsValid\n";
        }
    }

    // Test 4: ArmStateIsValid
    {
        test_count++;
        ArmState state;
        state.q = {0.0f, 1.0f, 2.0f};
        state.dq = {0.1f, 0.1f, 0.1f};
        state.tau = {0.0f, 0.0f, 0.0f};
        state.stamp = std::chrono::steady_clock::now();

        if (state.is_valid() && state.joint_count() == 3u) {
            passed++;
            std::cout << "✓ ArmStateIsValid\n";
        } else {
            std::cerr << "✗ ArmStateIsValid\n";
        }
    }

    // Test 5: ArmStateInvalidWhenSizesMismatch
    {
        test_count++;
        ArmState state;
        state.q = {0.0f, 1.0f, 2.0f};
        state.dq = {0.1f, 0.1f};
        state.tau = {0.0f, 0.0f, 0.0f};

        if (!state.is_valid()) {
            passed++;
            std::cout << "✓ ArmStateInvalidWhenSizesMismatch\n";
        } else {
            std::cerr << "✗ ArmStateInvalidWhenSizesMismatch\n";
        }
    }

    // Test 6: ArmStateAgeCalculation
    {
        test_count++;
        ArmState state;
        state.stamp = std::chrono::steady_clock::now();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        double age_ms = state.age_ms();

        if (age_ms >= 95.0 && age_ms <= 110.0) {
            passed++;
            std::cout << "✓ ArmStateAgeCalculation\n";
        } else {
            std::cerr << "✗ ArmStateAgeCalculation (age_ms=" << age_ms << ")\n";
        }
    }

    // Test 7: ArmStateFromBackendFlag
    {
        test_count++;
        ArmState state;
        state.q = {0.0f, 1.0f, 2.0f};
        state.dq = {0.1f, 0.1f, 0.1f};
        state.tau = {0.0f, 0.0f, 0.0f};
        state.stamp = std::chrono::steady_clock::now();
        state.from_backend = true;

        if (state.is_valid() && state.from_backend) {
            passed++;
            std::cout << "✓ ArmStateFromBackendFlag\n";
        } else {
            std::cerr << "✗ ArmStateFromBackendFlag\n";
        }
    }

    // Test 8: TransportStatsInitialValues
    {
        test_count++;
        TransportStats stats;

        if (stats.commands_sent == 0u &&
            stats.commands_failed == 0u &&
            stats.states_received == 0u &&
            stats.states_timeout == 0u &&
            stats.last_send_latency_ms == 0.0 &&
            stats.last_receive_latency_ms == 0.0) {
            passed++;
            std::cout << "✓ TransportStatsInitialValues\n";
        } else {
            std::cerr << "✗ TransportStatsInitialValues\n";
        }
    }

    // Test 9: TransportStatsSuccessRate
    {
        test_count++;
        TransportStats stats;

        double rate1 = stats.success_rate();
        stats.commands_sent = 8;
        stats.commands_failed = 2;
        double rate2 = stats.success_rate();

        if (rate1 == 0.0 && rate2 == 0.8) {
            passed++;
            std::cout << "✓ TransportStatsSuccessRate\n";
        } else {
            std::cerr << "✗ TransportStatsSuccessRate\n";
        }
    }

    // Test 10: IpcCommandPacketSerialization
    {
        test_count++;
        std::vector<float> q = {0.1f, 0.2f, 0.3f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f};
        std::vector<float> kp = {100.0f, 100.0f, 100.0f};
        std::vector<float> kd = {5.0f, 5.0f, 5.0f};
        std::vector<float> tau = {0.0f, 0.0f, 0.0f};

        auto packet = SerializeCommandPacket(3, q, dq, kp, kd, tau);

        if (packet[0] == 'G' && packet[1] == 'X' && packet[2] == '5' && packet[3] == 'C' &&
            packet.size() == 8 + 3 * 5 * 4) {
            passed++;
            std::cout << "✓ IpcCommandPacketSerialization\n";
        } else {
            std::cerr << "✗ IpcCommandPacketSerialization\n";
        }
    }

    // Test 11: IpcStatePacketSerialization
    {
        test_count++;
        std::vector<float> q = {0.1f, 0.2f, 0.3f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f};
        std::vector<float> tau = {0.5f, 0.6f, 0.7f};

        auto packet = SerializeStatePacket(3, true, q, dq, tau);

        if (packet[0] == 'G' && packet[1] == 'X' && packet[2] == '5' && packet[3] == 'S' &&
            packet.size() == 12 + 3 * 3 * 4) {
            passed++;
            std::cout << "✓ IpcStatePacketSerialization\n";
        } else {
            std::cerr << "✗ IpcStatePacketSerialization\n";
        }
    }

    // Test 12: IpcPosePacketSerialization
    {
        test_count++;
        std::vector<float> q = {0.1f, 0.2f, 0.3f};

        auto packet = SerializePosePacket(3, q);

        if (packet[0] == 'G' && packet[1] == 'X' && packet[2] == '5' && packet[3] == 'P' &&
            packet.size() == 8 + 3 * 4) {
            passed++;
            std::cout << "✓ IpcPosePacketSerialization\n";
        } else {
            std::cerr << "✗ IpcPosePacketSerialization\n";
        }
    }

    // Test 13: IpcCommandPacketParseRoundTrip
    {
        test_count++;
        std::vector<float> q = {0.1f, 0.2f, 0.3f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f};
        std::vector<float> kp = {100.0f, 100.0f, 100.0f};
        std::vector<float> kd = {5.0f, 5.0f, 5.0f};
        std::vector<float> tau = {0.5f, 0.6f, 0.7f};

        auto packet = SerializeCommandPacket(3, q, dq, kp, kd, tau);

        ArmCommandPacket parsed;
        std::string error;
        bool parse_ok = ParseCommandPacket(packet, parsed, &error);

        bool all_match = parse_ok && parsed.joint_count == 3u;
        if (all_match) {
            for (size_t i = 0; i < q.size(); ++i) {
                if (parsed.q[i] != q[i] || parsed.dq[i] != dq[i] ||
                    parsed.kp[i] != kp[i] || parsed.kd[i] != kd[i] ||
                    parsed.tau[i] != tau[i]) {
                    all_match = false;
                    break;
                }
            }
        }

        if (all_match) {
            passed++;
            std::cout << "✓ IpcCommandPacketParseRoundTrip\n";
        } else {
            std::cerr << "✗ IpcCommandPacketParseRoundTrip: " << error << "\n";
        }
    }

    // Test 14: IpcStatePacketParseRoundTrip
    {
        test_count++;
        std::vector<float> q = {0.1f, 0.2f, 0.3f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f};
        std::vector<float> tau = {0.5f, 0.6f, 0.7f};

        auto packet = SerializeStatePacket(3, true, q, dq, tau);

        ArmStatePacket parsed;
        std::string error;
        bool parse_ok = ParseStatePacket(packet, parsed, &error);

        bool all_match = parse_ok && parsed.joint_count == 3u && parsed.state_from_backend;
        if (all_match) {
            for (size_t i = 0; i < q.size(); ++i) {
                if (parsed.q[i] != q[i] || parsed.dq[i] != dq[i] || parsed.tau[i] != tau[i]) {
                    all_match = false;
                    break;
                }
            }
        }

        if (all_match) {
            passed++;
            std::cout << "✓ IpcStatePacketParseRoundTrip\n";
        } else {
            std::cerr << "✗ IpcStatePacketParseRoundTrip: " << error << "\n";
        }
    }

    // Test 15: IpcPosePacketParseRoundTrip
    {
        test_count++;
        std::vector<float> q = {0.1f, 0.2f, 0.3f};

        auto packet = SerializePosePacket(3, q);

        ArmPosePacket parsed;
        std::string error;
        bool parse_ok = ParsePosePacket(packet, parsed, &error);

        bool all_match = parse_ok && parsed.joint_count == 3u;
        if (all_match) {
            for (size_t i = 0; i < q.size(); ++i) {
                if (parsed.q[i] != q[i]) {
                    all_match = false;
                    break;
                }
            }
        }

        if (all_match) {
            passed++;
            std::cout << "✓ IpcPosePacketParseRoundTrip\n";
        } else {
            std::cerr << "✗ IpcPosePacketParseRoundTrip: " << error << "\n";
        }
    }

    // Test 16: IpcCommandPacketInvalidMagic
    {
        test_count++;
        std::vector<uint8_t> invalid_packet = {'X', 'X', '5', 'C', 0, 0, 0, 0};

        ArmCommandPacket parsed;
        std::string error;
        bool parse_ok = ParseCommandPacket(invalid_packet, parsed, &error);

        if (!parse_ok && error.find("magic") != std::string::npos) {
            passed++;
            std::cout << "✓ IpcCommandPacketInvalidMagic\n";
        } else {
            std::cerr << "✗ IpcCommandPacketInvalidMagic\n";
        }
    }

    // Test 17: IpcCommandPacketInvalidSize
    {
        test_count++;
        // Create a packet with header indicating 3 joints but only 8 bytes total
        // For 3 joints, expected size is 8 + 3*5*4 = 68 bytes
        std::vector<uint8_t> short_packet = {'G', 'X', '5', 'C', 1, 0, 3, 0};

        ArmCommandPacket parsed;
        std::string error;
        bool parse_ok = ParseCommandPacket(short_packet, parsed, &error);

        if (!parse_ok && error.find("size") != std::string::npos) {
            passed++;
            std::cout << "✓ IpcCommandPacketInvalidSize\n";
        } else {
            std::cerr << "✗ IpcCommandPacketInvalidSize: error=" << error << ", parse_ok=" << parse_ok << "\n";
        }
    }

    // Test 18: IpcNormalizeTransport
    {
        test_count++;
        if (NormalizeTransport("ROS") == "ros" &&
            NormalizeTransport("ipc") == "ipc" &&
            NormalizeTransport("TOPIC") == "ros" &&
            NormalizeTransport("UDP") == "udp") {
            passed++;
            std::cout << "✓ IpcNormalizeTransport\n";
        } else {
            std::cerr << "✗ IpcNormalizeTransport\n";
        }
    }

    // Test 19: IpcIsIpcTransport
    {
        test_count++;
        if (IsIpcTransport("ipc") &&
            IsIpcTransport("IPC") &&
            IsIpcTransport("Ipc") &&
            !IsIpcTransport("ros") &&
            !IsIpcTransport("topic")) {
            passed++;
            std::cout << "✓ IpcIsIpcTransport\n";
        } else {
            std::cerr << "✗ IpcIsIpcTransport\n";
        }
    }

    // Test 20: IpcTransportFactoryCreate
    {
        test_count++;
        auto transport = TransportFactory::CreateIpcTransport(45671, 45672, "127.0.0.1");

        if (transport && transport->GetMode() == TransportMode::IPC) {
            passed++;
            std::cout << "✓ IpcTransportFactoryCreate\n";
        } else {
            std::cerr << "✗ IpcTransportFactoryCreate\n";
        }
    }

    // Test 21: IpcTransportNotConnectedBeforeInit
    {
        test_count++;
        auto transport = TransportFactory::CreateIpcTransport(45673, 45674, "127.0.0.1");

        if (transport && !transport->IsConnected()) {
            passed++;
            std::cout << "✓ IpcTransportNotConnectedBeforeInit\n";
        } else {
            std::cerr << "✗ IpcTransportNotConnectedBeforeInit\n";
        }
    }

    // Test 22: IpcPacketSizeCalculations
    {
        test_count++;
        if (CommandPacketSize(6) == 8 + 6 * 5 * 4 &&
            StatePacketSize(6) == 12 + 6 * 3 * 4 &&
            PosePacketSize(6) == 8 + 6 * 4) {
            passed++;
            std::cout << "✓ IpcPacketSizeCalculations\n";
        } else {
            std::cerr << "✗ IpcPacketSizeCalculations\n";
        }
    }

    std::cout << "\n========================\n";
    std::cout << "Test Results: " << passed << "/" << test_count << " passed\n";

    return (passed == test_count) ? 0 : 1;
}
