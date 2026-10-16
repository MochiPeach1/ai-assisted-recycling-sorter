from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare("pa1_sortbot"), "worlds", "sort_world.world"]
    )
    model = PathJoinSubstitution(
        [FindPackageShare("pa1_sortbot"), "models", "sortbot", "model.sdf"]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "sortbot",
            "-file", model,
            "-x", "-1.45",
            "-y", "0.0",
            "-z", "0.08",
        ],
        output="screen",
    )
    sorter = Node(
        package="pa1_sortbot",
        executable="sorter",
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo GUI client",
            ),
            gazebo,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_robot, on_exit=[sorter])
            ),
        ]
    )
