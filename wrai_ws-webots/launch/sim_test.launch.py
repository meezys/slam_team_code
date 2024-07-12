import pathlib
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = (
        pathlib.Path(__file__).parent.parent.joinpath("config/sim_params.yml").resolve()
    )

    return LaunchDescription(
        [
            Node(
                package="wrai_mapping",
                executable="cone_slam",
                # remappings=[
                #     ("/cones", "/wrai/cones"),
                #     ("/tf", "/wrai/tf"),
                #     ("/tf_static", "/wrai/tf_static"),
                # ],
                parameters=[config],
            ),
            Node(
                package="wrai_path_planning",
                executable="midpoints",
                # remappings=[
                #     ("/cones", "/wrai/cones"),
                #     ("/tf", "/wrai/tf"),
                #     ("/tf_static", "/wrai/tf_static"),
                # ],
                parameters=[config],
            ),
            Node(
                package="wrai_vehicle_control",
                executable="pure_pursuit",
                # remappings=[("/tf", "/wrai/tf"), ("/tf_static", "/wrai/tf_static")],
                parameters=[config],
            ),
            GroupAction(
                actions=[
                    # SetRemap(src="/tf", dst="/wrai/tf"),
                    # SetRemap(src="/tf_static", dst="/wrai/tf_static"),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            [
                                FindPackageShare("wrai_mapping"),
                                "/launch",
                                "/transforms.launch.py",
                            ]
                        )
                    ),
                ]
            ),
            # ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen"),
        ]
    )
