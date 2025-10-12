import os
import turtle

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    bringup_dir = get_package_share_directory("nav2_edge_aware_planner")
    slam_dir = get_package_share_directory("slam_toolbox")
    turtlebot_dir = get_package_share_directory("turtlebot3_gazebo")
    launch_dir = os.path.join(bringup_dir, "launch")

    ld = LaunchDescription()

    edge_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "edge_navigation.launch.py")
        ),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    turtlebot_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_dir, "launch", "turtlebot3_house.launch.py")
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz")],
    )

    ld.add_action(edge_nav)
    ld.add_action(slam)
    ld.add_action(turtlebot_house)
    ld.add_action(rviz)

    return ld
