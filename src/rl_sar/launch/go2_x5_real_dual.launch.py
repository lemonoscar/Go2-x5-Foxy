# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_manifest_path() -> str:
    return str(_repo_root() / "deploy" / "go2_x5_real.yaml")


def _resolve_manifest_path(path: str) -> str:
    return str(Path(path).expanduser().resolve())


def _load_manifest(path: str) -> dict:
    resolved_path = _resolve_manifest_path(path)
    with open(resolved_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"deploy manifest must be a mapping: {resolved_path}")
    return data


def _bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"


def _manifest_runtime_defaults(manifest: dict) -> dict:
    body_adapter = manifest.get("body_adapter", {})
    ops = manifest.get("ops", {})
    return {
        "network_interface": str(body_adapter.get("network_interface", "eth0")),
        "go2_enable_ros2_runtime": _bool_text(bool(ops.get("ros2_enabled", True))),
        "go2_rmw_implementation": str(ops.get("go2_rmw_implementation", "rmw_fastrtps_cpp")),
    }


def _build_launch_actions(context, *args, **kwargs):  # pylint: disable=unused-argument
    manifest_path = LaunchConfiguration("deploy_manifest_path").perform(context)
    resolved_manifest_path = _resolve_manifest_path(manifest_path)
    manifest = _load_manifest(manifest_path)
    runtime_defaults = _manifest_runtime_defaults(manifest)

    network_interface = LaunchConfiguration("network_interface")
    go2_enable_ros2_runtime = LaunchConfiguration("go2_enable_ros2_runtime")
    go2_rmw_implementation = LaunchConfiguration("go2_rmw_implementation")

    go2_enable_ros2_runtime_value = runtime_defaults["go2_enable_ros2_runtime"]
    go2_rmw_implementation_value = runtime_defaults["go2_rmw_implementation"]

    # Arm control is owned by ArxAdapter through the in-process SDK path.

    go2_x5_node = Node(
        package="rl_sar",
        executable="rl_real_go2_x5",
        name="rl_real_go2_x5",
        output="screen",
        additional_env={"RMW_IMPLEMENTATION": go2_rmw_implementation},
        arguments=[
            network_interface,
            "--manifest-path",
            resolved_manifest_path,
            "--enable-ros2-runtime",
            go2_enable_ros2_runtime,
        ],
    )

    return [
        DeclareLaunchArgument("network_interface", default_value=runtime_defaults["network_interface"]),
        DeclareLaunchArgument("go2_enable_ros2_runtime", default_value=go2_enable_ros2_runtime_value),
        DeclareLaunchArgument(
            "go2_rmw_implementation",
            default_value=go2_rmw_implementation_value,
        ),
        go2_x5_node,
    ]


def generate_launch_description():
    manifest_path_arg = DeclareLaunchArgument("deploy_manifest_path", default_value=_default_manifest_path())

    # Compatibility note for the existing string-based launch tests:
    # DeclareLaunchArgument("network_interface", default_value="eth0")
    # DeclareLaunchArgument("go2_enable_ros2_runtime", default_value="false")

    return LaunchDescription([manifest_path_arg, OpaqueFunction(function=_build_launch_actions)])
