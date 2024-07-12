import pathlib
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        pathlib.Path(__file__).parent.parent.joinpath("config/car_params.yml").resolve()
    )

    return LaunchDescription(
        [
            Node(
                package="wrai_mapping",
                executable="cone_slam",
                emulate_tty=True,
                output="screen",
                parameters=[config],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    [
                        FindPackageShare("slam"),
                        "/launch",
                        "/transforms.launch.py",
                    ]
                )
            ),
        ]
    )
