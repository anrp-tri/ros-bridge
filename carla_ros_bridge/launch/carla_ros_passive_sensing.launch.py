from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import TextSubstitution
import launch_ros.actions


def generate_launch_description():
    # Get the object spawner node which creates the sensors we want.
    object_spawn_path = Path(get_package_share_directory("carla_spawn_objects"))
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str((object_spawn_path / "launch" / "carla_spawn_objects.launch.py").absolute())
        ),
        launch_arguments={
            "objects_definition_file": TextSubstitution(
                text=str((object_spawn_path / "config" / "object_sensor_only.json").absolute())
            ),
            "spawn_sensors_only": TextSubstitution(text="true"),
            "spawn_point_ego_vehicle": "dummy",
        }.items(),
    )
    ld = launch.LaunchDescription(
        [
            included_launch,
            launch.actions.DeclareLaunchArgument(
                name="host", default_value="localhost", description="IP of the CARLA server"
            ),
            launch.actions.DeclareLaunchArgument(
                name="port", default_value="2000", description="TCP port of the CARLA server"
            ),
            launch.actions.DeclareLaunchArgument(
                name="timeout",
                default_value="10",
                description="Time to wait for a successful connection to the CARLA server",
            ),
            launch.actions.DeclareLaunchArgument(
                name="synchronous_mode",
                default_value="True",
                description="Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors",
            ),
            launch.actions.DeclareLaunchArgument(
                name="fixed_delta_seconds",
                default_value="0.05",
                description="Simulation time (delta seconds) between simulation steps",
            ),
            launch.actions.DeclareLaunchArgument(
                name="ego_vehicle_role_name",
                default_value=[
                    "hero",
                    "ego_vehicle",
                    "hero0",
                    "hero1",
                    "hero2",
                    "hero3",
                    "hero4",
                    "hero5",
                    "hero6",
                    "hero7",
                    "hero8",
                    "hero9",
                ],
                description="Role names to identify ego vehicles. ",
            ),
            launch_ros.actions.Node(
                package="carla_ros_bridge",
                executable="bridge",
                name="carla_ros_bridge",
                output="screen",
                emulate_tty="True",
                on_exit=launch.actions.Shutdown(),
                parameters=[
                    # TODO(andrew.best): Perhaps this solves our problems w.r.t to time?
                    {"use_sim_time": True},
                    {"publish_clock": False},
                    {"host": launch.substitutions.LaunchConfiguration("host")},
                    {"port": launch.substitutions.LaunchConfiguration("port")},
                    {"timeout": launch.substitutions.LaunchConfiguration("timeout")},
                    {"passive": True},
                    {"synchronous_mode": launch.substitutions.LaunchConfiguration("synchronous_mode")},
                    {"synchronous_mode_wait_for_vehicle_control_command": False},
                    {"fixed_delta_seconds": launch.substitutions.LaunchConfiguration("fixed_delta_seconds")},
                    {"register_all_sensors": True},
                    {"ego_vehicle_role_name": launch.substitutions.LaunchConfiguration("ego_vehicle_role_name")},
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
