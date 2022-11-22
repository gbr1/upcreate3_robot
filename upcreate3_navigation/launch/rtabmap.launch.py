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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    rtabmap_namespace = 'rtabmap'
    rtabmap_ns_prefix = '/'+rtabmap_namespace

    qos = 2

    parameters=[{
        'frame_id':'camera_link',
        'subscribe_depth':True,
        'subscribe_scan':False,
        'approx_sync':True,
        'Reg/Strategy':'1',
        'Reg/Force3DoF':'true',
        'RGBD/NeighborLinkRefining':'True',
        'Grid/RangeMin':'0.2',
        'Optimizer/GravitySigma':'0',
        'qos_odom':qos,
        'qos_image':qos}
    ]

    remappings=[
        (rtabmap_ns_prefix+'/rgb/image', '/camera/color/image_raw'),
        (rtabmap_ns_prefix+'/rgb/camera_info', '/camera/color/camera_info'),
        (rtabmap_ns_prefix+'/depth/image', '/camera/aligned_depth_to_color/image_raw')
    ]

    rtabmap_specific_remappings=[
        (rtabmap_ns_prefix+'/odom','/odom')
    ]

    #vo_specific_remappings=[
    #    ('odom','/visual_odometry') 
    #]

    vo_node = Node(
        package='rtabmap_ros', executable='rgbd_odometry', output='screen',
        namespace=rtabmap_namespace,
        parameters=parameters,
        remappings=remappings #+vo_specific_remappings
    )

    rgbd_sync_node =  Node(
            package='rtabmap_ros', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=remappings+rtabmap_specific_remappings
    )

    rtabmap_node = Node(
        package='rtabmap_ros', executable='rtabmap', output='screen',
        namespace=rtabmap_namespace,
        parameters=parameters,
        remappings=remappings +rtabmap_specific_remappings,
        arguments=['-d']
    )

    ld = LaunchDescription()
    #ld.add_action(vo_node)
    #ld.add_action(rgbd_sync_node)
    ld.add_action(rtabmap_node)


    return ld