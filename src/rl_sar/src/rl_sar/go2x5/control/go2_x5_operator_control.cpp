#include "rl_sar/go2x5/operator_control.hpp"

namespace Go2X5OperatorControl
{

Config BuildConfig(const YamlParams& params, const RLConfig::LayeredGo2X5ConfigState& layered_config)
{
    Config config;
    if (layered_config.IsActive())
    {
        const auto& typed = layered_config.Typed();
        config.real_deploy_exclusive_keyboard_control = typed.operator_config.real_deploy_exclusive_keyboard_control;
        config.key1_prefer_navigation_mode = typed.operator_config.key1_prefer_navigation_mode;
        config.fixed_cmd_x = typed.operator_config.fixed_cmd_x;
        config.fixed_cmd_y = typed.operator_config.fixed_cmd_y;
        config.fixed_cmd_yaw = typed.operator_config.fixed_cmd_yaw;
        const YAML::Node& merged = layered_config.MergedNode();
        config.has_fixed_cmd_x = merged["fixed_cmd_x"].IsDefined();
        config.has_fixed_cmd_y = merged["fixed_cmd_y"].IsDefined();
        config.has_fixed_cmd_yaw = merged["fixed_cmd_yaw"].IsDefined();
        return config;
    }

    config.real_deploy_exclusive_keyboard_control =
        params.Get<bool>("real_deploy_exclusive_keyboard_control", false);
    config.key1_prefer_navigation_mode = params.Get<bool>("key1_prefer_navigation_mode", false);
    config.has_fixed_cmd_x = params.Has("fixed_cmd_x");
    config.has_fixed_cmd_y = params.Has("fixed_cmd_y");
    config.has_fixed_cmd_yaw = params.Has("fixed_cmd_yaw");
    config.fixed_cmd_x = params.Get<float>("fixed_cmd_x", 0.0f);
    config.fixed_cmd_y = params.Get<float>("fixed_cmd_y", 0.0f);
    config.fixed_cmd_yaw = params.Get<float>("fixed_cmd_yaw", 0.0f);
    return config;
}

bool IsExclusiveGo2X5Control(const std::string& robot_name, const Config& config)
{
    return robot_name == "go2_x5" && config.real_deploy_exclusive_keyboard_control;
}

bool ShouldTriggerKey1(const Control& control, const std::string& robot_name, bool in_rl_locomotion)
{
    return robot_name == "go2_x5" && in_rl_locomotion &&
           control.current_keyboard == Input::Keyboard::Num1 &&
           control.last_keyboard != Input::Keyboard::Num1;
}

Go2X5ControlLogic::Key1Mode ResolveKey1Mode(const Config& config)
{
    return Go2X5ControlLogic::ResolveKey1Mode(config.key1_prefer_navigation_mode);
}

void ApplyFixedCommand(Control* control, const Config& config, float default_x, float default_y, float default_yaw)
{
    control->navigation_mode = false;
        control->x = config.has_fixed_cmd_x ? config.fixed_cmd_x : default_x;
    control->y = config.has_fixed_cmd_y ? config.fixed_cmd_y : default_y;
    control->yaw = config.has_fixed_cmd_yaw ? config.fixed_cmd_yaw : default_yaw;
}

void ApplyNavigationMode(Control* control)
{
    control->navigation_mode = true;
    control->x = 0.0f;
    control->y = 0.0f;
    control->yaw = 0.0f;
}

void ApplyManualVelocityKey(Control* control, Input::Keyboard key)
{
    switch (key)
    {
    case Input::Keyboard::W: control->x += 0.1f; break;
    case Input::Keyboard::S: control->x -= 0.1f; break;
    case Input::Keyboard::A: control->y += 0.1f; break;
    case Input::Keyboard::D: control->y -= 0.1f; break;
    case Input::Keyboard::Q: control->yaw += 0.1f; break;
    case Input::Keyboard::E: control->yaw -= 0.1f; break;
    default: break;
    }
}

bool IsStopKey(Input::Keyboard key)
{
    return key == Input::Keyboard::Space;
}

bool IsZeroKey(Input::Keyboard key)
{
    return key == Input::Keyboard::Num5;
}

bool IsNavKeyboardDown(const Control& control, bool exclusive_control)
{
    return !exclusive_control && control.current_keyboard == Input::Keyboard::N;
}

bool IsNavGamepadDown(const Control& control, bool exclusive_control)
{
    return !exclusive_control && control.current_gamepad == Input::Gamepad::X;
}

bool ShouldToggleNavigation(const Control& control, bool exclusive_control)
{
    const bool nav_keyboard_down = IsNavKeyboardDown(control, exclusive_control);
    const bool nav_gamepad_down = IsNavGamepadDown(control, exclusive_control);
    return (nav_keyboard_down && !control.nav_keyboard_latched) ||
           (nav_gamepad_down && !control.nav_gamepad_latched);
}

} // namespace Go2X5OperatorControl
