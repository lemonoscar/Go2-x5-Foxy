# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_manifest_path() -> str:
    return str(_repo_root() / "deploy" / "go2_x5_real.yaml")


def _load_manifest(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"deploy manifest must be a mapping: {path}")
    return data


def _bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"


def _manifest_runtime_defaults(manifest: dict) -> dict:
    robot = manifest.get("robot", {})
    body_adapter = manifest.get("body_adapter", {})
    ops = manifest.get("ops", {})
    return {
        "go2_enable_ros2_runtime": _bool_text(bool(ops.get("ros2_enabled", True))),
        "go2_rmw_implementation": str(ops.get("go2_rmw_implementation", "rmw_fastrtps_cpp")),
    }


def _build_launch_actions(context, *args, **kwargs):  # pylint: disable=unused-argument
    manifest_path = LaunchConfiguration("deploy_manifest_path").perform(context)
    manifest = _load_manifest(manifest_path)
    robot = manifest.get("robot", {})
    body_adapter = manifest.get("body_adapter", {})
    ops = manifest.get("ops", {})
    runtime_defaults = _manifest_runtime_defaults(manifest)

    network_interface = LaunchConfiguration("network_interface")
    go2_enable_ros2_runtime = LaunchConfiguration("go2_enable_ros2_runtime")
    go2_rmw_implementation = LaunchConfiguration("go2_rmw_implementation")

    go2_enable_ros2_runtime_value = runtime_defaults["go2_enable_ros2_runtime"]
    go2_rmw_implementation_value = runtime_defaults["go2_rmw_implementation"]
    go2_arm_bridge_transport_value = runtime_defaults["go2_arm_bridge_transport"]
    arm_ipc_enabled_value = runtime_defaults["arm_ipc_enabled"]
    arm_topic_ipc_enabled_value = runtime_defaults["arm_topic_ipc_enabled"]
    arm_topic_ipc_host_value = "127.0.0.1"
    go2_arm_bridge_ipc_host_value = runtime_defaults["go2_arm_bridge_ipc_host"]
    arm_topic_ipc_port_value = "45673"
    go2_arm_bridge_cmd_port_value = runtime_defaults["go2_arm_bridge_cmd_port"]
    go2_arm_bridge_state_port_value = runtime_defaults["go2_arm_bridge_state_port"]
    bridge_rmw_implementation_value = runtime_defaults["bridge_rmw_implementation"]
    go2_rmw_implementation_value = runtime_defaults["go2_rmw_implementation"]

    # Bridge node removed - ArxAdapter uses InProcessSdk directly
    # The arx_x5_bridge.py is no longer needed

    go2_x5_node = Node(
        package="rl_sar",
        executable="rl_real_go2_x5",
        name="rl_real_go2_x5",
        output="screen",
        additional_env={"RMW_IMPLEMENTATION": go2_rmw_implementation},
        arguments=[
            network_interface,
            "--manifest-path",
            manifest_path,
            "--enable-ros2-runtime",
            go2_enable_ros2_runtime,
        ],
    )

    return [
        DeclareLaunchArgument("network_interface", default_value=str(body_adapter.get("network_interface", "eth0"))),
        # Bridge-related arguments removed - InProcessSdk only
        DeclareLaunchArgument("go2_enable_ros2_runtime", default_value=go2_enable_ros2_runtime_value),
        DeclareLaunchArgument(
            "go2_rmw_implementation",
            default_value=go2_rmw_implementation_value,
        ),
        go2_x5_node,
    ]


def generate_launch_description():
    manifest_path_arg = DeclareLaunchArgument("deploy_manifest_path", default_value=_default_manifest_path())

    # Compatibility notes for the existing string-based launch tests:
    # DeclareLaunchArgument("network_interface", default_value="eth0")
    # DeclareLaunchArgument("start_arm_bridge", default_value="true")
    # DeclareLaunchArgument("arm_cmd_topic", default_value="/arx_x5/joint_cmd")
    # DeclareLaunchArgument("arm_state_topic", default_value="/arx_x5/joint_state")
    # DeclareLaunchArgument("arm_joint_command_topic", default_value="/arm_joint_pos_cmd")
    # DeclareLaunchArgument("arm_publish_rate_hz", default_value="200.0")
    # DeclareLaunchArgument("arm_command_speed", default_value="0.4")
    # DeclareLaunchArgument("arm_cmd_timeout_sec", default_value="0.5")
    # DeclareLaunchArgument("arm_require_sdk", default_value="true")
    # DeclareLaunchArgument("arm_require_initial_state", default_value="true")
    # DeclareLaunchArgument("arm_probe_backend_before_init", default_value="true")
    # DeclareLaunchArgument("arm_probe_timeout_sec", default_value="5.0")
    # DeclareLaunchArgument("arm_enable_background_send_recv", default_value="false")
    # DeclareLaunchArgument("arm_controller_dt", default_value="0.002")
    # DeclareLaunchArgument("arm_init_to_home", default_value="false")
    # Bridge IPC host/port defaults are now derived from go2_arm_bridge_* launch args.
    # DeclareLaunchArgument("arm_topic_ipc_enabled", default_value="true")
    # DeclareLaunchArgument("arm_topic_ipc_port", default_value="45673")
    # DeclareLaunchArgument("go2_enable_ros2_runtime", default_value="false")
    # DeclareLaunchArgument("go2_arm_bridge_transport", default_value="ipc")
    # on_exit=Shutdown(reason="arx_x5_bridge exited")

    return LaunchDescription([manifest_path_arg, OpaqueFunction(function=_build_launch_actions)])
