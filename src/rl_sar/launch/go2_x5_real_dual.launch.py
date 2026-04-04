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
    arm_adapter = manifest.get("arm_adapter", {})
    coordinator = manifest.get("coordinator", {})
    supervisor = manifest.get("supervisor", {})
    ops = manifest.get("ops", {})
    bridge_transport = "ipc" if ops.get("ros2_mirror_only", True) else "ros"
    return {
        "arm_joint_count": str(robot.get("arm_joint_count", 6)),
        "arm_cmd_topic": str(arm_adapter.get("arm_cmd_topic", "/arx_x5/joint_cmd")),
        "arm_state_topic": str(arm_adapter.get("arm_state_topic", "/arx_x5/joint_state")),
        "arm_joint_command_topic": str(arm_adapter.get("arm_joint_command_topic", "/arm_joint_pos_cmd")),
        "arm_publish_rate_hz": str(arm_adapter.get("arm_target_rate_hz", 200)),
        "arm_cmd_timeout_sec": f"{float(coordinator.get('arm_command_expire_ms', 15)) / 1000.0:.3f}".rstrip("0").rstrip("."),
        "arm_enable_background_send_recv": _bool_text(bool(arm_adapter.get("background_send_recv", True))),
        "arm_controller_dt": f"{float(arm_adapter.get('controller_dt', 0.002)):.3f}",
        "arm_probe_timeout_sec": f"{float(supervisor.get('probe_window_sec', 2.0)):.1f}",
        "go2_enable_ros2_runtime": _bool_text(bool(ops.get("ros2_enabled", True))),
        "go2_arm_bridge_transport": bridge_transport,
        "bridge_rmw_implementation": str(ops.get("bridge_rmw_implementation", "rmw_cyclonedds_cpp")),
        "go2_rmw_implementation": str(ops.get("go2_rmw_implementation", "rmw_fastrtps_cpp")),
        "arm_ipc_enabled": _bool_text(bridge_transport == "ipc"),
        "arm_topic_ipc_enabled": _bool_text(bridge_transport == "ipc"),
        "go2_arm_bridge_ipc_host": "127.0.0.1",
        "go2_arm_bridge_cmd_port": "45671",
        "go2_arm_bridge_state_port": "45672",
    }


def _build_launch_actions(context, *args, **kwargs):  # pylint: disable=unused-argument
    manifest_path = LaunchConfiguration("deploy_manifest_path").perform(context)
    manifest = _load_manifest(manifest_path)
    robot = manifest.get("robot", {})
    body_adapter = manifest.get("body_adapter", {})
    arm_adapter = manifest.get("arm_adapter", {})
    coordinator = manifest.get("coordinator", {})
    supervisor = manifest.get("supervisor", {})
    ops = manifest.get("ops", {})
    runtime_defaults = _manifest_runtime_defaults(manifest)

    network_interface = LaunchConfiguration("network_interface")
    start_arm_bridge = LaunchConfiguration("start_arm_bridge")
    arm_model = LaunchConfiguration("arm_model")
    arm_interface_name = LaunchConfiguration("arm_interface_name")
    arm_urdf_path = LaunchConfiguration("arm_urdf_path")
    arm_joint_count = LaunchConfiguration("arm_joint_count")
    arm_cmd_topic = LaunchConfiguration("arm_cmd_topic")
    arm_state_topic = LaunchConfiguration("arm_state_topic")
    arm_joint_command_topic = LaunchConfiguration("arm_joint_command_topic")
    arm_publish_rate_hz = LaunchConfiguration("arm_publish_rate_hz")
    arm_command_speed = LaunchConfiguration("arm_command_speed")
    arm_cmd_timeout_sec = LaunchConfiguration("arm_cmd_timeout_sec")
    arm_accept_commands = LaunchConfiguration("arm_accept_commands")
    arm_dry_run = LaunchConfiguration("arm_dry_run")
    arm_sdk_root = LaunchConfiguration("arm_sdk_root")
    arm_sdk_python_path = LaunchConfiguration("arm_sdk_python_path")
    arm_sdk_lib_path = LaunchConfiguration("arm_sdk_lib_path")
    arm_require_sdk = LaunchConfiguration("arm_require_sdk")
    arm_require_initial_state = LaunchConfiguration("arm_require_initial_state")
    arm_probe_backend_before_init = LaunchConfiguration("arm_probe_backend_before_init")
    arm_probe_timeout_sec = LaunchConfiguration("arm_probe_timeout_sec")
    arm_enable_background_send_recv = LaunchConfiguration("arm_enable_background_send_recv")
    arm_controller_dt = LaunchConfiguration("arm_controller_dt")
    arm_init_to_home = LaunchConfiguration("arm_init_to_home")
    arm_ipc_enabled = LaunchConfiguration("arm_ipc_enabled")
    arm_topic_ipc_enabled = LaunchConfiguration("arm_topic_ipc_enabled")
    arm_topic_ipc_host = LaunchConfiguration("arm_topic_ipc_host")
    arm_topic_ipc_port = LaunchConfiguration("arm_topic_ipc_port")
    go2_enable_ros2_runtime = LaunchConfiguration("go2_enable_ros2_runtime")
    go2_arm_bridge_transport = LaunchConfiguration("go2_arm_bridge_transport")
    go2_arm_bridge_ipc_host = LaunchConfiguration("go2_arm_bridge_ipc_host")
    go2_arm_bridge_cmd_port = LaunchConfiguration("go2_arm_bridge_cmd_port")
    go2_arm_bridge_state_port = LaunchConfiguration("go2_arm_bridge_state_port")
    bridge_rmw_implementation = LaunchConfiguration("bridge_rmw_implementation")
    go2_rmw_implementation = LaunchConfiguration("go2_rmw_implementation")

    arm_target_rate_hz = runtime_defaults["arm_publish_rate_hz"]
    arm_cmd_topic_value = runtime_defaults["arm_cmd_topic"]
    arm_state_topic_value = runtime_defaults["arm_state_topic"]
    arm_joint_command_topic_value = runtime_defaults["arm_joint_command_topic"]
    arm_controller_dt_value = runtime_defaults["arm_controller_dt"]
    arm_probe_timeout_sec_value = runtime_defaults["arm_probe_timeout_sec"]
    arm_cmd_timeout_sec_value = runtime_defaults["arm_cmd_timeout_sec"]
    arm_enable_background_send_recv_value = runtime_defaults["arm_enable_background_send_recv"]
    go2_enable_ros2_runtime_value = runtime_defaults["go2_enable_ros2_runtime"]
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

    arm_bridge_node = Node(
        package="rl_sar",
        executable="arx_x5_bridge.py",
        name="arx_x5_bridge",
        output="screen",
        condition=IfCondition(start_arm_bridge),
        on_exit=Shutdown(reason="arx_x5_bridge exited"),
        additional_env={"RMW_IMPLEMENTATION": bridge_rmw_implementation},
        parameters=[
            {
                "deploy_manifest_path": manifest_path,
                "bridge_transport": go2_arm_bridge_transport,
                "model": arm_model,
                "interface_name": arm_interface_name,
                "urdf_path": arm_urdf_path,
                "joint_count": arm_joint_count,
                "cmd_topic": arm_cmd_topic,
                "state_topic": arm_state_topic,
                "arm_joint_command_topic": arm_joint_command_topic,
                "publish_rate_hz": arm_publish_rate_hz,
                "command_speed": arm_command_speed,
                "cmd_timeout_sec": arm_cmd_timeout_sec,
                "accept_commands": arm_accept_commands,
                "dry_run": arm_dry_run,
                "sdk_root": arm_sdk_root,
                "sdk_python_path": arm_sdk_python_path,
                "sdk_lib_path": arm_sdk_lib_path,
                "require_sdk": arm_require_sdk,
                "require_initial_state": arm_require_initial_state,
                "probe_backend_before_init": arm_probe_backend_before_init,
                "probe_timeout_sec": arm_probe_timeout_sec,
                "enable_background_send_recv": arm_enable_background_send_recv,
                "controller_dt": arm_controller_dt,
                "init_to_home": arm_init_to_home,
                "ipc_enabled": arm_ipc_enabled,
                "ipc_bind_host": go2_arm_bridge_ipc_host,
                "ipc_cmd_port": go2_arm_bridge_cmd_port,
                "ipc_state_host": go2_arm_bridge_ipc_host,
                "ipc_state_port": go2_arm_bridge_state_port,
                "arm_topic_ipc_enabled": arm_topic_ipc_enabled,
                "arm_topic_ipc_host": arm_topic_ipc_host,
                "arm_topic_ipc_port": arm_topic_ipc_port,
            }
        ],
    )

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
            "--arm-bridge-transport",
            go2_arm_bridge_transport,
            "--arm-bridge-ipc-host",
            go2_arm_bridge_ipc_host,
            "--arm-bridge-cmd-port",
            go2_arm_bridge_cmd_port,
            "--arm-bridge-state-port",
            go2_arm_bridge_state_port,
            "--arm-joint-cmd-port",
            arm_topic_ipc_port,
        ],
    )

    return [
        DeclareLaunchArgument("network_interface", default_value=str(body_adapter.get("network_interface", "eth0"))),
        DeclareLaunchArgument("start_arm_bridge", default_value="true"),
        DeclareLaunchArgument("arm_model", default_value="X5"),
        DeclareLaunchArgument("arm_interface_name", default_value=str(arm_adapter.get("can_interface", "can0"))),
        DeclareLaunchArgument("arm_urdf_path", default_value=""),
        DeclareLaunchArgument("arm_joint_count", default_value=runtime_defaults["arm_joint_count"]),
        DeclareLaunchArgument("arm_cmd_topic", default_value=arm_cmd_topic_value),
        DeclareLaunchArgument("arm_state_topic", default_value=arm_state_topic_value),
        DeclareLaunchArgument("arm_joint_command_topic", default_value=arm_joint_command_topic_value),
        DeclareLaunchArgument("arm_publish_rate_hz", default_value=arm_target_rate_hz),
        DeclareLaunchArgument("arm_command_speed", default_value="0.4"),
        DeclareLaunchArgument("arm_cmd_timeout_sec", default_value=arm_cmd_timeout_sec_value),
        DeclareLaunchArgument("arm_accept_commands", default_value="true"),
        DeclareLaunchArgument("arm_dry_run", default_value="false"),
        DeclareLaunchArgument("arm_sdk_root", default_value=EnvironmentVariable("ARX5_SDK_ROOT", default_value="")),
        DeclareLaunchArgument("arm_sdk_python_path", default_value=EnvironmentVariable("ARX5_SDK_PYTHON_PATH", default_value="")),
        DeclareLaunchArgument("arm_sdk_lib_path", default_value=EnvironmentVariable("ARX5_SDK_LIB_PATH", default_value="")),
        DeclareLaunchArgument("arm_require_sdk", default_value="true"),
        DeclareLaunchArgument("arm_require_initial_state", default_value="true"),
        DeclareLaunchArgument("arm_probe_backend_before_init", default_value="true"),
        DeclareLaunchArgument("arm_probe_timeout_sec", default_value=arm_probe_timeout_sec_value),
        DeclareLaunchArgument("arm_enable_background_send_recv", default_value=arm_enable_background_send_recv_value),
        DeclareLaunchArgument("arm_controller_dt", default_value=arm_controller_dt_value),
        DeclareLaunchArgument("arm_init_to_home", default_value="false"),
        DeclareLaunchArgument("arm_ipc_enabled", default_value=arm_ipc_enabled_value),
        DeclareLaunchArgument("arm_topic_ipc_enabled", default_value=arm_topic_ipc_enabled_value),
        DeclareLaunchArgument("arm_topic_ipc_host", default_value=arm_topic_ipc_host_value),
        DeclareLaunchArgument("arm_topic_ipc_port", default_value=arm_topic_ipc_port_value),
        DeclareLaunchArgument("go2_enable_ros2_runtime", default_value=go2_enable_ros2_runtime_value),
        DeclareLaunchArgument("go2_arm_bridge_transport", default_value=go2_arm_bridge_transport_value),
        DeclareLaunchArgument("go2_arm_bridge_ipc_host", default_value=go2_arm_bridge_ipc_host_value),
        DeclareLaunchArgument("go2_arm_bridge_cmd_port", default_value=go2_arm_bridge_cmd_port_value),
        DeclareLaunchArgument("go2_arm_bridge_state_port", default_value=go2_arm_bridge_state_port_value),
        DeclareLaunchArgument(
            "bridge_rmw_implementation",
            default_value=bridge_rmw_implementation_value,
        ),
        DeclareLaunchArgument(
            "go2_rmw_implementation",
            default_value=go2_rmw_implementation_value,
        ),
        arm_bridge_node,
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
