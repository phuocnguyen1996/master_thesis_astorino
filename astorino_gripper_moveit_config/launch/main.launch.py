import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    demo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('astorino_gripper_moveit_config'), 'launch'),
            '/demo.launch.py'])
        )
    # camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('astorino_gripper_moveit_config'), 'launch'),
    #         '/realsense_pose.launch.py'])
    #     )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('astorino_gripper_moveit_config'), 'launch'),
            '/camera_pose.launch.py'])
        )
    return LaunchDescription([demo_moveit, camera])
