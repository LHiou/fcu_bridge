from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=EnvironmentVariable('FCU_BRIDGE_CONFIG', default_value=PathJoinSubstitution([
                FindPackageShare('fcu_bridge'), 'config', 'bridge.yaml',
            ])),
        ),
        DeclareLaunchArgument(
            'namespace', default_value=EnvironmentVariable('UAV_NAMESPACE', default_value='uav1'),
        ),
        DeclareLaunchArgument(
            'use_sim_time', choices=['true', 'false'],
            default_value=EnvironmentVariable('USE_SIM_TIME', default_value='false'),
        ),
        Node(
            package='fcu_bridge',
            executable='bridge',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                LaunchConfiguration('config'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            output='screen',
        ),
    ])
