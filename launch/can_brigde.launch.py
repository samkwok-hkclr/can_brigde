import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    
    launch_dir = os.path.join(get_package_share_directory("can_brigde"), "launch")
    
    range1 = [f"0x50{i}:7FF" for i in range(1, 8)]
    range2 = [f"0x70{i}:7FF" for i in range(1, 8)]

    dual_arm_ids = ",".join(range1 + range2)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "dual_arm_can_brigde.launch.py")
            ),
            launch_arguments={
                "enable_can_fd": "True",
                "namespace": "upper_orin",
                "filters": dual_arm_ids
            }.items(),
        )
    )
    
    eef_ids = "0x1D5:7FF,0x2D5:7FF,0x3D5:7FF,0x4D5:7FF,0x755:7FF"
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "eef_can_brigde.launch.py")
            ),
            launch_arguments={
                "enable_can_fd": "False",
                "namespace": "upper_orin",
                "filters": eef_ids,
            }.items(),
        )
    )
    
    return ld