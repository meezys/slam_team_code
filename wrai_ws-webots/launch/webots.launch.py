import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('webots_fsai')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    # simulation environment
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'simple_trackdrive.wbt'),
        ros2_supervisor=True
    )

    webots_driver = WebotsController(
        robot_name='fsaivehicle',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    with open( robot_description_path, "r" ) as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        webots_driver,
        robot_state_publisher_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
