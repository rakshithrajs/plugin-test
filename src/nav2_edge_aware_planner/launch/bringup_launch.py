import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    bringup_dir = get_package_share_directory("nav2_edge_aware_planner")
    slam_dir = get_package_share_directory("slam_toolbox")
    launch_dir = os.path.join(bringup_dir, "launch")
    rviz_dir = os.path.join(bringup_dir, "rviz")
    turtlebot3_house = get_package_share_directory("turtlebot3_gazebo")

    nodes_started = set()

    def start_next_node(event, context):
        print(f"Node {event.action.name} has started")
        nodes_started.add(event.action.name)

        if nodes_started >= {"slam_toolbox", "edge_navigation", "gazebo"}:
            print("All required nodes launched, starting RViz")
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(rviz_dir, "nav2_default_view.rviz")],
            )
            context.launch_description.add_action(rviz_node)

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
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_house, "launch", "turtlebot3_house.launch.py")
        )
    )

    slam_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
            ),
            on_start=start_next_node,
        )
    )

    edge_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=Node(
                package="nav2_edge_aware_planner",
                executable="edge_navigation",
                name="edge_navigation",
            ),
            on_start=start_next_node,
        )
    )

    gazebo_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=Node(
                package="gazebo_ros",
                executable="gzserver",
                name="gazebo",
            ),
            on_start=start_next_node,
        )
    )

    ld.add_action(edge_nav)
    ld.add_action(slam)
    ld.add_action(gazebo)
    ld.add_action(slam_event)
    ld.add_action(edge_event)
    ld.add_action(gazebo_event)

    return ld
