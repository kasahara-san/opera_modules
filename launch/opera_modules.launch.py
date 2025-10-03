import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction

def generate_launch_description():

    opera_modules_dir=get_package_share_directory("opera_modules")
    params = LaunchConfiguration('params_file', default=os.path.join(opera_modules_dir, 'params', 'params.yaml'))

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        GroupAction([
            Node(
                package='opera_modules',
                executable='soil_release_module',
                namespace='ic120'
            ),
            Node(
                package='opera_modules',
                executable='watchdog_ekf_input',
                parameters = 
                    [
                        params,
                        {
                            'use_sim_time': bool(LaunchConfiguration('use_sim_time'))
                        }
                    ]
            ),
            Node(
                package='opera_modules',
                executable='topic_relay_gui',
                parameters = 
                    [
                        params,
                        {
                            'use_sim_time': bool(LaunchConfiguration('use_sim_time'))
                        }
                    ]
            ),
        ])
    ])