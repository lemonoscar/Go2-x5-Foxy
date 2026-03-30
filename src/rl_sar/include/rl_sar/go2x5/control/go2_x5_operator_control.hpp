#ifndef GO2_X5_OPERATOR_CONTROL_HPP
#define GO2_X5_OPERATOR_CONTROL_HPP

#include <string>

#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"
#include "rl_sdk.hpp"

namespace Go2X5OperatorControl
{

struct Config
{
    bool real_deploy_exclusive_keyboard_control = false;
    bool key1_prefer_navigation_mode = false;
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
Go2X5ControlLogic::Key1Mode ResolveKey1Mode(const Config& config);
void ApplyFixedCommand(Control* control, const Config& config, float default_x, float default_y, float default_yaw);
void ApplyNavigationMode(Control* control);
void ApplyManualVelocityKey(Control* control, Input::Keyboard key);
bool IsStopKey(Input::Keyboard key);
bool IsZeroKey(Input::Keyboard key);
bool IsNavKeyboardDown(const Control& control, bool exclusive_control);
bool IsNavGamepadDown(const Control& control, bool exclusive_control);
bool ShouldToggleNavigation(const Control& control, bool exclusive_control);

} // namespace Go2X5OperatorControl

#endif // GO2_X5_OPERATOR_CONTROL_HPP
