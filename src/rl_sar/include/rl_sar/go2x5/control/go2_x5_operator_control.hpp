#ifndef GO2_X5_OPERATOR_CONTROL_HPP
#define GO2_X5_OPERATOR_CONTROL_HPP

#include <string>

#include "rl_sdk.hpp"

namespace Go2X5OperatorControl
{

struct Config
{
    bool real_deploy_exclusive_keyboard_control = false;
    float fixed_cmd_x = 0.0f;
    float fixed_cmd_y = 0.0f;
    float fixed_cmd_yaw = 0.0f;
    bool has_fixed_cmd_x = false;
    bool has_fixed_cmd_y = false;
    bool has_fixed_cmd_yaw = false;
};

Config BuildConfig(const YamlParams& params, const RLConfig::LayeredGo2X5ConfigState& layered_config);
bool IsExclusiveGo2X5Control(const std::string& robot_name, const Config& config);
bool ShouldTriggerKey1(const Control& control, const std::string& robot_name, bool in_rl_locomotion);
void ApplyFixedCommand(Control* control, const Config& config, float default_x, float default_y, float default_yaw);
bool IsStopKey(Input::Keyboard key);

} // namespace Go2X5OperatorControl

#endif // GO2_X5_OPERATOR_CONTROL_HPP
