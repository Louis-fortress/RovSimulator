"""
upload_reconrobot.launch.py
============================
ROS2 launch file to spawn the ReconRobot AUV into UUV Simulator / Gazebo.
Place this file in:
  uuv_descriptions/launch/upload_reconrobot.launch.py

Usage:
  ros2 launch uuv_descriptions upload_reconrobot.launch.py
  ros2 launch uuv_descriptions upload_reconrobot.launch.py x:=0 y:=0 z:=-20 namespace:=reconrobot

To launch with a world first:
  ros2 launch uuv_gazebo_worlds ocean_waves.launch.py
  ros2 launch uuv_descriptions upload_reconrobot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from plankton_utils.time import is_sim_time
import pathlib
import xacro

def spawn_robot(context, *args, **kwargs):
    # Resolve all launch arguments to strings
    namespace  = LaunchConfiguration('namespace').perform(context)
    x          = LaunchConfiguration('x').perform(context)
    y          = LaunchConfiguration('y').perform(context)
    z          = LaunchConfiguration('z').perform(context)
    roll       = LaunchConfiguration('roll').perform(context)
    pitch      = LaunchConfiguration('pitch').perform(context)
    yaw        = LaunchConfiguration('yaw').perform(context)
    use_ned    = LaunchConfiguration('use_ned_frame').perform(context)
    debug      = LaunchConfiguration('debug').perform(context)

    # Locate the xacro file inside uuv_descriptions package
    pkg_dir = get_package_share_directory('uuv_descriptions')
    xacro_file = os.path.join(pkg_dir, 'robots', 'reconrobot_default.xacro')
    res = is_sim_time(return_param=False, use_subprocess=True)

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(
            f'\n[upload_reconrobot] Cannot find xacro file at:\n  {xacro_file}\n'
            f'Make sure reconrobot_default.xacro is in the robots/ folder of\n'
            f'uuv_descriptions and you have run: colcon build --packages-select uuv_descriptions'
        )
    
    # Process xacro → URDF string
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'namespace':                namespace,
            'debug':                    debug,
            'inertial_reference_frame': 'world',
        }
    )
    robot_desc = doc.toxml()

    # Node 1: robot_state_publisher — publishes TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )

    map_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        parameters=[{'use_sim_time': True}],
    )

#     message_to_tf_node = Node(
#         package='uuv_assistants',
#         executable='uuv_message_to_tf',
#         name='message_to_tf',
#         namespace=namespace,
#         output='screen',
#         parameters=[{
#             'use_sim_time': True,
#             'odometry_topic': 'pose_gt',
#             'world_frame':    'world',
#             'child_frame_id': f'/{namespace}/base_link',
#         }]
# )

    world_ned_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_frame_publisher',
        arguments=['0', '0', '0',
                '1.5707963267948966', '0', '3.141592653589793',
                'world', 'world_ned'],
        parameters=[{'use_sim_time': True}],
        prefix="bash -c 'sleep 5; $0 $@'"
        )

    #Gui for controlling the thrusters using sliders
    joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='reconrobot',
            parameters=[{
                'robot_description': robot_desc,
            }]
    )

    # Node 2: spawn_entity — spawns the robot into Gazebo
    spawn_args = [
        '-entity', namespace,
        '-x', x,
        '-y', y,
        '-z', z,
        '-R', roll,
        '-P', pitch,
        '-Y', yaw,
        '-topic', f'/{namespace}/robot_description',
    ]

    if debug == '1':
        spawn_args += ['-verbose']

    # Message to tf
    message_to_tf_launch = os.path.join(
        get_package_share_directory('uuv_assistants'),
        'launch',
        'message_to_tf.launch'
    )

    if not pathlib.Path(message_to_tf_launch).exists():
        exc = 'Launch file ' + message_to_tf_launch + ' does not exist'
        raise Exception(exc)

    launch_args = [('namespace', namespace), ('world_frame', 'world'), 
            ('child_frame_id', '/' + namespace + '/base_link'), ('use_sim_time', str(res).lower()),]
    message_to_tf_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(message_to_tf_launch), launch_arguments=launch_args)

    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='reconrobot_spawner',
        output='screen',
        arguments=spawn_args,
    )

    return [robot_state_publisher, spawner, map_to_world, world_ned_tf, message_to_tf_launch]

def generate_launch_description():
    return LaunchDescription([

        # ---- Launch arguments (mirrors rexrov convention) ----
        DeclareLaunchArgument(
            'debug', default_value='0',
            description='Enable verbose Gazebo output'),

        DeclareLaunchArgument(
            'x', default_value='0',
            description='Initial X position (m)'),

        DeclareLaunchArgument(
            'y', default_value='0',
            description='Initial Y position (m)'),

        DeclareLaunchArgument(
            'z', default_value='0',
            description='Initial Z position (m) — negative = underwater'),

        DeclareLaunchArgument(
            'roll', default_value='0.0',
            description='Initial roll (rad)'),

        DeclareLaunchArgument(
            'pitch', default_value='0.0',
            description='Initial pitch (rad)'),

        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Initial yaw (rad)'),

        DeclareLaunchArgument(
            'namespace', default_value='reconrobot',
            description='Robot namespace — prefixes all ROS topics'),

        DeclareLaunchArgument(
            'use_ned_frame', default_value='false',
            description='Use NED frame instead of ENU (default ENU)'),

        # ---- Spawn action ----
        OpaqueFunction(function=spawn_robot),
    ])