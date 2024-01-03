from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression

from os import environ
from os import pathsep
from os import path

from ament_index_python.packages import get_package_share_path, get_package_prefix


def prepare_paths(plugins_path, models_path):
    resources_path = models_path

    home_path = path.expanduser("~")

    # Adding paths as in /usr/share/gazebo-11/setup.sh
    # so the default world empty.world (and other standard elements) are accesible
    base_gazebo11 = "/usr/share/gazebo-11/"
    models_path += pathsep + base_gazebo11 + "models"
    resources_path += pathsep + base_gazebo11
    plugins_path += pathsep + base_gazebo11 + "plugins"

    # Add models downloaded by gazebo and stored in home directory
    models_path += pathsep + home_path + "/.gazebo/models"

    # PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash
    # TODO: Compile libraries in this workspace
    # Temporary fix!
    models_path += (
        pathsep
        + home_path
        + "/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
    )
    plugins_path += (
        pathsep
        + home_path
        + "/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic"
    )

    ld_library_path = plugins_path

    if "GAZEBO_MODEL_PATH" in environ:
        models_path += pathsep + environ["GAZEBO_MODEL_PATH"]
    if "GAZEBO_PLUGIN_PATH" in environ:
        plugins_path += pathsep + environ["GAZEBO_PLUGIN_PATH"]
    if "GAZEBO_RESOURCE_PATH" in environ:
        resources_path += pathsep + environ["GAZEBO_RESOURCE_PATH"]
    if "LD_LIBRARY_PATH" in environ:
        ld_library_path += pathsep + environ["LD_LIBRARY_PATH"]

    return {
        "GAZEBO_MODEL_PATH": models_path,
        "GAZEBO_RESOURCE_PATH": resources_path,
        "GAZEBO_PLUGIN_PATH": plugins_path,
        "LD_LIBRARY_PATH": ld_library_path,
        # "GAZEBO_MODEL_DATABASE_URI": "",
        # "LIBGL_ALWAYS_SOFTWARE": "1", # On intel gpus & wsl2
    }


def generate_launch_description():
    package_name = "gazebo_custom_plugins"
    package_install_path = get_package_prefix(package_name)
    plugins_path = path.join(package_install_path, "lib")
    models_path = path.join(get_package_share_path(package_name), "models")
    worlds_path = path.join(get_package_share_path(package_name), "worlds")

    env_paths = prepare_paths(plugins_path, models_path)

    # Gazebo server, no UI just computing core.
    start_gz_server = ExecuteProcess(
        cmd=["gzserver", "--verbose", worlds_path + "/tof_demo.world"],
        output="screen",
        additional_env=env_paths,  # type: ignore
    )

    headless = LaunchConfiguration("headless")

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="0"),
            start_gz_server,
            # Gazebo client - UI, no computing
            ExecuteProcess(
                condition=IfCondition(PythonExpression([headless, " == 0"])),
                cmd=["gzclient", "--verbose"],
                additional_env=env_paths,  # type: ignore
                shell=True,
            ),
        ]
    )  # )]
