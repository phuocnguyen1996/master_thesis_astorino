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
                "0.460305",
                "--y",
                "0.604874",
                "--z",
                "0.159666",
                "--qx",
                "0.0532723",
                "--qy",
                "0.0167902",
                "--qz",
                "-0.760187",
                "--qw",
                "0.647299",
                # "--roll",
                # "0.094802",
                # "--pitch",
                # "-0.0592919",
                # "--yaw",
                # "-1.72805",
            ],
        ),
    ]
    return LaunchDescription(nodes)
