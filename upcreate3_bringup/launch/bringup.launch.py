from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    # upcreate3 folders
    upcreate3_bringup_path = get_package_share_directory('upcreate3_bringup')
    upcreate3_description_path = get_package_share_directory('upcreate3_description')
    upcreate3_navigation_path = get_package_share_directory('upcreate3_navigation')

    # description launcher
    description_launch_file = PathJoinSubstitution(
        [upcreate3_description_path, 'launch', 'description.launch.py']
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file)
    )


    # include folder in launch directory
    include_path = PathJoinSubstitution(
        [upcreate3_bringup_path, 'launch', 'include'])

    # realsense launcher
    realsense_launch_file = PathJoinSubstitution(
        [include_path, '.', 'realsense.launch.py']
    )
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )


    # rtabmap launcher
    rtabmap_launch_file = PathJoinSubstitution(
        [upcreate3_navigation_path, 'launch', 'rtabmap.launch.py']
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file)
    )




    # Launch Description
    ld = LaunchDescription()
    ld.add_action(realsense_launch)
    ld.add_action(description_launch)
    ld.add_action(rtabmap_launch)
    return ld