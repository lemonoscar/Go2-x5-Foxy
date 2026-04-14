#include <array>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

struct OD_Motor_Msg
{
    int16_t current_actual_int = 0;
    float speed_actual_rad = 0.0f;
    float angle_actual_rad = 0.0f;
    uint16_t motor_id = 0;
    uint8_t temperature = 0;
    uint8_t error = 0;
    float angle_actual_float = 0.0f;
    float current_actual_float = 0.0f;
    float gripper_pos = 0.0f;
    float gripper_spd = 0.0f;
    float gripper_cur = 0.0f;
    float gripper_last_pos = 0.0f;
    float gripper_totalangle = 0.0f;
    float round_cnt = 0.0f;
};

namespace
{

struct FakeHardwareState
{
    std::array<OD_Motor_Msg, 10> motor_msg{};
};

std::mutex& FakeMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::unordered_map<const void*, FakeHardwareState>& FakeStates()
{
    static std::unordered_map<const void*, FakeHardwareState> states;
    return states;
}

void InitializeMotor(OD_Motor_Msg& msg, const uint16_t motor_id, const float q)
{
    msg.motor_id = motor_id;
    msg.angle_actual_rad = q;
    msg.angle_actual_float = q;
    msg.speed_actual_rad = 0.0f;
    msg.current_actual_float = 0.0f;
    msg.temperature = 25u;
    msg.error = 0u;
}

}  // namespace

class ArxCan
{
public:
    explicit ArxCan(std::string interface_name);
    ~ArxCan();

    void can_cmd_init(uint16_t motor_id, uint8_t cmd);
    void send_EC_motor_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    void query_EC_motor_pos(uint16_t motor_id);
    void query_EC_motor_vel(uint16_t motor_id);
    void query_EC_motor_current(uint16_t motor_id);
    void set_motor(uint16_t motor_id, uint8_t cmd);
    void send_DM_motor_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    void enable_DM_motor(uint16_t motor_id);
    void reset_zero_readout(uint16_t motor_id);
    void clear(uint16_t motor_id);
    std::array<OD_Motor_Msg, 10> get_motor_msg();
};

ArxCan::ArxCan(std::string interface_name)
{
    if (interface_name.empty())
    {
        throw std::runtime_error("fake ARX interface name must not be empty");
    }

    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    InitializeMotor(state.motor_msg[1], 1, 0.11f);
    InitializeMotor(state.motor_msg[2], 2, -0.22f);
    InitializeMotor(state.motor_msg[4], 4, 0.33f);
    InitializeMotor(state.motor_msg[5], 5, -0.44f);
    InitializeMotor(state.motor_msg[6], 6, 0.55f);
    InitializeMotor(state.motor_msg[7], 7, -0.66f);
    InitializeMotor(state.motor_msg[8], 8, 0.0f);
}

ArxCan::~ArxCan()
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    FakeStates().erase(this);
}

void ArxCan::can_cmd_init(uint16_t motor_id, uint8_t)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id].motor_id = motor_id;
    }
}

void ArxCan::send_EC_motor_cmd(uint16_t motor_id, float, float, float pos, float spd, float tor)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id >= state.motor_msg.size())
    {
        return;
    }
    auto& msg = state.motor_msg[motor_id];
    msg.motor_id = motor_id;
    msg.angle_actual_rad = pos;
    msg.angle_actual_float = pos;
    msg.speed_actual_rad = spd;
    msg.current_actual_float = tor;
}

void ArxCan::query_EC_motor_pos(uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id].motor_id = motor_id;
    }
}

void ArxCan::query_EC_motor_vel(uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id].motor_id = motor_id;
    }
}

void ArxCan::query_EC_motor_current(uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id].motor_id = motor_id;
    }
}

void ArxCan::set_motor(uint16_t motor_id, uint8_t)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id].motor_id = motor_id;
    }
}

void ArxCan::send_DM_motor_cmd(uint16_t motor_id, float, float, float pos, float spd, float tor)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id >= state.motor_msg.size())
    {
        return;
    }
    auto& msg = state.motor_msg[motor_id];
    msg.motor_id = motor_id;
    msg.angle_actual_rad = pos;
    msg.angle_actual_float = pos;
    msg.speed_actual_rad = spd;
    msg.current_actual_float = tor;
}

void ArxCan::enable_DM_motor(uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id].motor_id = motor_id;
    }
}

void ArxCan::reset_zero_readout(uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id >= state.motor_msg.size())
    {
        return;
    }
    state.motor_msg[motor_id].angle_actual_rad = 0.0f;
    state.motor_msg[motor_id].angle_actual_float = 0.0f;
}

void ArxCan::clear(uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    auto& state = FakeStates()[this];
    if (motor_id < state.motor_msg.size())
    {
        state.motor_msg[motor_id] = OD_Motor_Msg{};
    }
}

std::array<OD_Motor_Msg, 10> ArxCan::get_motor_msg()
{
    std::lock_guard<std::mutex> lock(FakeMutex());
    return FakeStates()[this].motor_msg;
}
