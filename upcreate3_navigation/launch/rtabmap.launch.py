from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    rtabmap_namespace = 'rtabmap'
    rtabmap_ns_prefix = '/'+rtabmap_namespace

    parameters=[{
        'frame_id':'camera_link',
        'subscribe_depth':True,
        'approx_sync':False,
        'Reg/Strategy':'1',
        'Reg/Force3DoF':'true',
        'RGBD/NeighborLinkRefining':'True',
        'Grid/RangeMin':'0.2',
        'Optimizer/GravitySigma':'0'}
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

    rtabmap_node = Node(
        package='rtabmap_ros', executable='rtabmap', output='screen',
        namespace=rtabmap_namespace,
        parameters=parameters,
        remappings=remappings +rtabmap_specific_remappings,
        arguments=['-d']
    )

    ld = LaunchDescription()
    ld.add_action(vo_node)
    ld.add_action(rtabmap_node)


    return ld