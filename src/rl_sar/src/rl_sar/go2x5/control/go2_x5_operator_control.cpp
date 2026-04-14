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
    (void)in_rl_locomotion;
    return robot_name == "go2_x5" &&
           control.current_keyboard == Input::Keyboard::Num1 &&
           control.last_keyboard != Input::Keyboard::Num1;
}

void ApplyFixedCommand(Control* control, const Config& config, float default_x, float default_y, float default_yaw)
{
    control->x = config.has_fixed_cmd_x ? config.fixed_cmd_x : default_x;
    control->y = config.has_fixed_cmd_y ? config.fixed_cmd_y : default_y;
    control->yaw = config.has_fixed_cmd_yaw ? config.fixed_cmd_yaw : default_yaw;
}

bool IsStopKey(Input::Keyboard key)
{
    return key == Input::Keyboard::Space;
}

} // namespace Go2X5OperatorControl
