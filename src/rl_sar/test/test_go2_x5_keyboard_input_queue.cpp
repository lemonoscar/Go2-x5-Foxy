#include <gtest/gtest.h>

#include "rl_sdk.hpp"

namespace rl_sar::test
{

namespace
{

class TestRL final : public RL
{
public:
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

}  // namespace

TEST(Go2X5KeyboardInputQueueTest, PreservesEventOrderAcrossMultipleInputs)
{
    TestRL rl;

    rl.EnqueueKeyboardInput(Input::Keyboard::Num0);
    rl.EnqueueKeyboardInput(Input::Keyboard::Num1);
    rl.EnqueueKeyboardInput(Input::Keyboard::Num2);

    ASSERT_EQ(rl.PendingKeyboardInputCount(), 3u);
    EXPECT_EQ(rl.ConsumePendingKeyboardInput(), Input::Keyboard::Num0);
    EXPECT_EQ(rl.ConsumePendingKeyboardInput(), Input::Keyboard::Num1);
    EXPECT_EQ(rl.ConsumePendingKeyboardInput(), Input::Keyboard::Num2);
    EXPECT_EQ(rl.ConsumePendingKeyboardInput(), Input::Keyboard::None);
    EXPECT_EQ(rl.PendingKeyboardInputCount(), 0u);
}

TEST(Go2X5KeyboardInputQueueTest, IgnoresNoneEvents)
{
    TestRL rl;

    rl.EnqueueKeyboardInput(Input::Keyboard::None);
    rl.EnqueueKeyboardInput(Input::Keyboard::Space);

    ASSERT_EQ(rl.PendingKeyboardInputCount(), 1u);
    EXPECT_EQ(rl.ConsumePendingKeyboardInput(), Input::Keyboard::Space);
    EXPECT_EQ(rl.ConsumePendingKeyboardInput(), Input::Keyboard::None);
}

}  // namespace rl_sar::test
