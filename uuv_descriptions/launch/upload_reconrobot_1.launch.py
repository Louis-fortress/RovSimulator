from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration as Lc

import launch_testing.actions

from ament_index_python.packages import get_package_share_directory

import os
import pathlib
import xacro

from plankton_utils.time import is_sim_time


def launch_setup(context, *args, **kwargs):
    #Perform substitutions
    debug = Lc('debug').perfor(context)
    namespace = Lc('namespace').perform(context)
    x = Lc('x').perform(context)
    y = Lc('y').perform(context)
    z = Lc('z').perform(context)
    roll = Lc('roll').perform(context)
    pitch = Lc('pitch').perform(context)
    yaw = Lc('yaw').perform(context)
    use_world_ned = Lc('use_ned_frame').perform(context)
    is_write_on_disk = Lc('write_file_on_disk').perform(context)

    #Request sim time value to the global node
    res = is_sim_time(return_param=False, use_subprocess=True)

    #Xacro
    xacro_file = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'reconrobot_default.xacro'
    )

    #Check for existence of file
    path = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'generated',
        namespace,
    )

    if not pathlib.Path(path).exists():
        try:
            #Create directories
            os.makedirs(path)
        except OSError:
            print ("Creation of the directory %s failed" % path)
    