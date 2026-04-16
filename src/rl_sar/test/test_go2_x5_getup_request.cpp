#include <gtest/gtest.h>

#include "fsm_go2_x5.hpp"
#include "rl_sdk.hpp"

namespace rl_sar::test
{

namespace
{

class TestRL final : public RL
{
public:
    bool init_rl_should_throw = false;
    int init_rl_call_count = 0;
    std::string last_init_rl_path;

    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
    void InitRL(std::string robot_config_path) override
    {
        ++init_rl_call_count;
        last_init_rl_path = std::move(robot_config_path);
        if (init_rl_should_throw)
        {
            throw std::runtime_error("test init failure");
        }

        rl_init_done = true;
    }
};

class DummyState final : public FSMState
{
public:
    explicit DummyState(const std::string& name) : FSMState(name) {}
    void Enter() override {}
    void Run() override {}
    void Exit() override {}
};

void ConfigureTestRl(TestRL* rl)
{
    rl->robot_name = "go2_x5";
    rl->params.config_node["num_of_dofs"] = 18;
    rl->params.config_node["default_dof_pos"] = std::vector<float>(18, 0.0f);
    rl->params.config_node["fixed_kp"] = std::vector<float>(18, 20.0f);
    rl->params.config_node["fixed_kd"] = std::vector<float>(18, 1.0f);
    rl->params.config_node["rl_kp"] = std::vector<float>(18, 20.0f);
    rl->params.config_node["rl_kd"] = std::vector<float>(18, 1.0f);
    rl->params.config_node["dt"] = 0.02f;
    rl->InitJointNum(18);
}

}  // namespace

TEST(Go2X5GetUpRequestTest, KeyZeroTransitionsIntoGetUpEvenAfterInputClears)
{
    TestRL rl;
    ConfigureTestRl(&rl);

    auto fsm = FSMManager::GetInstance().CreateFSM("go2_x5", &rl);
    ASSERT_NE(fsm, nullptr);
    rl.fsm = *fsm;

    ASSERT_NE(rl.fsm.current_state_, nullptr);
    EXPECT_EQ(rl.fsm.current_state_->GetStateName(), "RLFSMStatePassive");

    rl.EnqueueKeyboardInput(Input::Keyboard::Num0);
    rl.StateController(&rl.robot_state, &rl.robot_command);
    rl.control.ClearInput();

    EXPECT_EQ(rl.fsm.current_state_->GetStateName(), "RLFSMStatePassive");

    rl.StateController(&rl.robot_state, &rl.robot_command);

    ASSERT_NE(rl.fsm.current_state_, nullptr);
    EXPECT_EQ(rl.fsm.current_state_->GetStateName(), "RLFSMStateGetUp");
}

TEST(Go2X5GetUpRequestTest, ClearInputDropsKeyboardLatch)
{
    Control control;
    control.SetKeyboard(Input::Keyboard::Num1);
    control.ClearInput();

    EXPECT_EQ(control.current_keyboard, Input::Keyboard::None);
    EXPECT_EQ(control.current_gamepad, Input::Gamepad::None);
}

TEST(Go2X5GetUpRequestTest, EnterRlRequestPersistsUntilGetUpCompletes)
{
    TestRL rl;
    ConfigureTestRl(&rl);
    rl.fsm.previous_state_ = std::make_shared<DummyState>("RLFSMStatePassive");
    rl.now_state = rl.robot_state;
    rl.start_state = rl.robot_state;

    go2_x5_fsm::RLFSMStateGetUp get_up_state(&rl);
    get_up_state.fsm_state = &rl.robot_state;
    get_up_state.fsm_command = &rl.robot_command;
    get_up_state.Enter();

    rl.RequestEnterRl();

    for (int i = 0; i < 200; ++i)
    {
        get_up_state.Run();
    }

    EXPECT_EQ(get_up_state.CheckChange(), "RLFSMStateRLLocomotion");
    EXPECT_FALSE(rl.ConsumeEnterRlRequest());
}

TEST(Go2X5GetUpRequestTest, RlLocomotionEnterStaysInRlWhenInitSucceeds)
{
    TestRL rl;
    ConfigureTestRl(&rl);
    rl.fsm.previous_state_ = std::make_shared<DummyState>("RLFSMStateGetUp");
    rl.now_state = rl.robot_state;
    rl.start_state = rl.robot_state;

    auto passive = std::make_shared<DummyState>("RLFSMStatePassive");
    auto locomotion = std::make_shared<go2_x5_fsm::RLFSMStateRLLocomotion>(&rl);
    rl.fsm.current_state_ = passive;
    rl.fsm.next_state_ = locomotion;
    rl.fsm.states_["RLFSMStatePassive"] = passive;
    rl.fsm.states_["RLFSMStateRLLocomotion"] = locomotion;
    rl.fsm.mode_ = FSM::Mode::CHANGE;

    rl.StateController(&rl.robot_state, &rl.robot_command);

    ASSERT_NE(rl.fsm.current_state_, nullptr);
    EXPECT_EQ(rl.fsm.current_state_->GetStateName(), "RLFSMStateRLLocomotion");
    EXPECT_EQ(rl.init_rl_call_count, 1);
    EXPECT_EQ(rl.last_init_rl_path, "go2_x5/robot_lab");
    EXPECT_TRUE(rl.rl_init_done);
    EXPECT_EQ(rl.fsm.mode_, FSM::Mode::NORMAL);
}

TEST(Go2X5GetUpRequestTest, KeyOneDoesNotInjectForwardVelocityOnRlEntryRequest)
{
    TestRL rl;
    ConfigureTestRl(&rl);

    auto fsm = FSMManager::GetInstance().CreateFSM("go2_x5", &rl);
    ASSERT_NE(fsm, nullptr);
    rl.fsm = *fsm;
    rl.control.x = 0.6f;
    rl.control.y = -0.2f;
    rl.control.yaw = 0.1f;

    rl.EnqueueKeyboardInput(Input::Keyboard::Num1);
    rl.StateController(&rl.robot_state, &rl.robot_command);

    EXPECT_FLOAT_EQ(rl.control.x, 0.0f);
    EXPECT_FLOAT_EQ(rl.control.y, 0.0f);
    EXPECT_FLOAT_EQ(rl.control.yaw, 0.0f);
}

TEST(Go2X5GetUpRequestTest, RlLocomotionEnterFallsBackToPassiveWhenInitFails)
{
    TestRL rl;
    ConfigureTestRl(&rl);
    rl.init_rl_should_throw = true;
    rl.fsm.previous_state_ = std::make_shared<DummyState>("RLFSMStateGetUp");
    rl.now_state = rl.robot_state;
    rl.start_state = rl.robot_state;

    auto passive = std::make_shared<DummyState>("RLFSMStatePassive");
    auto locomotion = std::make_shared<go2_x5_fsm::RLFSMStateRLLocomotion>(&rl);
    rl.fsm.current_state_ = passive;
    rl.fsm.next_state_ = locomotion;
    rl.fsm.states_["RLFSMStatePassive"] = passive;
    rl.fsm.states_["RLFSMStateRLLocomotion"] = locomotion;
    rl.fsm.mode_ = FSM::Mode::CHANGE;

    rl.StateController(&rl.robot_state, &rl.robot_command);

    ASSERT_NE(rl.fsm.current_state_, nullptr);
    EXPECT_EQ(rl.fsm.current_state_->GetStateName(), "RLFSMStateRLLocomotion");
    EXPECT_EQ(rl.fsm.mode_, FSM::Mode::CHANGE);
    EXPECT_FALSE(rl.rl_init_done);

    rl.StateController(&rl.robot_state, &rl.robot_command);

    ASSERT_NE(rl.fsm.current_state_, nullptr);
    EXPECT_EQ(rl.fsm.current_state_->GetStateName(), "RLFSMStatePassive");
}

}  // namespace rl_sar::test
