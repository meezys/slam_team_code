from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="wrai_perception",
                executable="yolo",
                emulate_tty=True,
                output="screen",
            ),
            Node(
                package="wrai_mapping",
                executable="cone_slam",
                emulate_tty=True,
                output="screen",
            ),
            Node(
                package="wrai_path_planning",
                executable="midpoints",
                emulate_tty=True,
                output="screen",
            ),
            Node(
                package="wrai_vehicle_control",
                executable="pure_pursuit",
                emulate_tty=True,
                output="screen",
            ),
            Node(
                package="wrai_mapping",
                executable="chassis_tf",
            ),
            # IncludeLaunchDescription(
            #     AnyLaunchDescriptionSource(
            #         [
            #             FindPackageShare("rosbridge_server"),
            #             "/launch",
            #             "/rosbridge_websocket_launch.xml",
            #         ]
            #     )
            # ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    [
                        FindPackageShare("vectornav"),
                        "/launch",
                        "/vectornav.launch.py",
                    ]
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "chassis",
                    "--child-frame-id",
                    "camera",
                ],
            ),
            ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen"),
        ]
    )
