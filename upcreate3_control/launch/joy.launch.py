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
from launch.substitutions import TextSubstitution
import os




def generate_launch_description():
    # upcreate3 joy config
    upcreate3_joy_path = get_package_share_directory('upcreate3_control')
    joy_config_file = PathJoinSubstitution(
        [upcreate3_joy_path, 'config', 'joy.yaml']
    )


    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])

    config_filepath=LaunchConfiguration('config_filepath', default=joy_config_file)
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath],
            remappings=[('cmd_vel', 'joy_cmd_vel')]
        )


    # Launch joy teleop
    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    return ld