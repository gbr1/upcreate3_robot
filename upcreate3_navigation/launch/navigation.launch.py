# The MIT License
# 
# Copyright (c) 2022 Giovanni di Dio Bruno https://gbr1.github.io
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # standard bringup used in nav2 guides
    navigation_bringup_path = get_package_share_directory('nav2_bringup')
    upcreate3_navigation_path = get_package_share_directory('upcreate3_navigation')

    # description launcher
    navigation_bringup_launch_file = PathJoinSubstitution(
        [navigation_bringup_path, 'launch', 'navigation_launch.py']
    )

    navigation_config_file = PathJoinSubstitution(
        [upcreate3_navigation_path, 'config', 'nav.yaml']
    )


    navigation_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_bringup_launch_file),
        launch_arguments = {'params_file': navigation_config_file,
                           }.items()
    )


    # Launch Description
    ld = LaunchDescription()
    ld.add_action(navigation_bringup_launch)
    return ld