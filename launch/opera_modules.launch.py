import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

robot_name="ic120"
use_namespace=True

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('robot_name', default_value='ic120'),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name
            ),
            Node(
                package='opera_modules',
                executable='soil_release_module',
                name='soil_release_module_for_ic120'
            ),
        ])
    ])