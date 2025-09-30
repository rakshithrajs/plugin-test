from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Packages
    tb3_desc_pkg = get_package_share_directory("turtlebot3_description")
    slam_toolbox_pkg = get_package_share_directory("slam_toolbox")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")

    # URDF file
    urdf_file = os.path.join(
        tb3_desc_pkg, "urdf", "turtlebot3_" + os.environ["TURTLEBOT3_MODEL"] + ".urdf"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[urdf_file],
    )

    # SLAM Toolbox (async online mode)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, "launch", "online_async_launch.py")
        )
    )

    # Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "use_rviz": "false",
            "slam": "true",
        }.items(),
    )

    # RViz2 with Nav2 config
    rviz_config = os.path.join(nav2_bringup_pkg, "rviz", "nav2_default_view.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            slam,
            nav2,
            rviz2,
        ]
    )
