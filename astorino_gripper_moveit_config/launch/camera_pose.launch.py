""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> camera_link """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "camera_link",
                "--x",
                "0.36305",
                "--y",
                "0.717154",
                "--z",
                "0.079186",
                "--qx",
                "0.00472739",
                "--qy",
                "0.00395202",
                "--qz",
                "-0.699338",
                "--qw",
                "0.714765",
                # "--roll",
                # "0.0122858",
                # "--pitch",
                # "-0.00096256",
                # "--yaw",
                # "-1.54897",
            ],
        ),
    ]
    return LaunchDescription(nodes)
