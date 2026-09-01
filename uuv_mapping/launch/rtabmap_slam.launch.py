from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('uuv_mapping')
    config = os.path.join(pkg_share, 'config', 'rtabmap_config.yaml')

    rtabmap_ros = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[config,{'use_sim_time': True}],
        remappings=[
            ('scan_cloud', '/reconrobot/sonar'),
            ('odom', '/reconrobot/pose_gt'),
        ],
        arguments=['-d']
    )

    water_surface_filter = Node(
        package='pcl_ros',
        executable='filter_passthrough_node',
        name='sonar_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'filter_field_name': 'z',
            'filter_limit_min': -50.0,   # below robot
            'filter_limit_max':  0.5,    # just above robot (cuts water surface)
        }],
        remappings=[
            ('input',  '/reconrobot/sonar'),
            ('output', '/reconrobot/sonar_filtered'),
        ]
    )

    rtabmap_rviz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[config,
            {'use_sim_time': True}],
        remappings=[
            ('scan_cloud', '/reconrobot/sonar'),
            ('odom', '/reconrobot/pose_gt'),
        ]
    )

    return LaunchDescription([
        rtabmap_ros,
        #water_surface_filter,
        rtabmap_rviz
    ])

