import os
import tempfile
from pathlib import Path

import yaml
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Domain the bridge configs were written for: the rest of the OARBot system.
CONFIG_SYSTEM_DOMAIN = 0


def retarget_domain(config: dict, system_domain: int) -> dict:
    """Return a copy of a domain_bridge config with the system side moved to system_domain.

    Each Dingo keeps its own fixed domain; only references to the system domain change.
    """
    def move(value):
        return system_domain if value == CONFIG_SYSTEM_DOMAIN else value

    out = dict(config)
    for key in ("from_domain", "to_domain"):
        if key in out:
            out[key] = move(out[key])
    out["topics"] = {
        name: {k: (move(v) if k in ("from_domain", "to_domain") else v) for k, v in spec.items()}
        for name, spec in config.get("topics", {}).items()
    }
    return out


def generate_domain_bridge_node(context: LaunchContext, *args, **kwargs) -> list[Node]:
    oarbot_name_text = LaunchConfiguration("oarbot_name").perform(context)
    config_path = Path(FindPackageShare("oarbot_launch").perform(context)) / "config" / f"{oarbot_name_text}_dingo_bridge.yaml"
    system_domain = int(os.environ.get("ROS_DOMAIN_ID") or CONFIG_SYSTEM_DOMAIN)

    if system_domain != CONFIG_SYSTEM_DOMAIN:
        original = yaml.safe_load(config_path.read_text())
        dingo_domain = original["from_domain"]
        if system_domain == dingo_domain:
            raise RuntimeError(f"ROS_DOMAIN_ID={system_domain} is {oarbot_name_text}'s Dingo domain; pick another")
        config = retarget_domain(original, system_domain)
        handle = tempfile.NamedTemporaryFile("w", prefix=f"{oarbot_name_text}_dingo_bridge_d{system_domain}_", suffix=".yaml", delete=False)
        with handle:
            yaml.safe_dump(config, handle, sort_keys=False)
        config_path = Path(handle.name)

    return [
        Node(
            package="domain_bridge",
            executable="domain_bridge",
            name="domain_bridge",
            namespace=oarbot_name_text,
            arguments=[str(config_path)]
        )
    ]


def generate_launch_description() -> LaunchDescription:
    oarbot_name_string = "oarbot_name"
    oarbot_name_argument = DeclareLaunchArgument(oarbot_name_string, description="Name of the oarbot; should be one of: oarbot_blue, oarbot_silver")

    return LaunchDescription([
        oarbot_name_argument,
        OpaqueFunction(function=generate_domain_bridge_node)
    ])
