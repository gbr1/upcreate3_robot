from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    upcreate3_bringup_path = get_package_share_directory('upcreate3_bringup')
    realsense_path = get_package_share_directory('realsense2_camera')

    realsense_launch_file = PathJoinSubstitution(
        [realsense_path, 'launch', 'rs_launch.py'])
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments = {'align_depth.enable': 'true',
                            'initial_reset'     : 'true',
                            'pointcloud.enable' : 'false',
                           }.items()
    )

    ld = LaunchDescription()
    ld.add_action(realsense_launch)
    return ld