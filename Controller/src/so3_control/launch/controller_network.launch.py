import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_logger_dir():
    logger_dir = os.path.join(os.path.expanduser('~'), '.ros', 'so3_control', 'logger')
    os.makedirs(logger_dir, exist_ok=True)
    return logger_dir + os.sep


def generate_launch_description():
    # Declare launch arguments
    hover_thrust = LaunchConfiguration('hover_thrust', default='0.38')
    logger_dir = _default_logger_dir()

    # Declare launch arguments
    hover_thrust_arg = DeclareLaunchArgument(
        'hover_thrust',
        default_value='0.38',
        description='Hover thrust value'
    )

    # Network controller node
    network_controller_node = Node(
        package='so3_control',
        executable='network_control_node',
        name='network_controller_node',
        output='screen',
        parameters=[
            {
                'is_simulation': False,
                'use_disturbance_observer': True,
                'hover_thrust': hover_thrust,
                'kx_xy': 5.7,
                'kx_z': 6.2,
                'kv_xy': 3.4,
                'kv_z': 4.0,
                'record_log': True,
                'logger_file_name': logger_dir
            }
        ],
        remappings=[
            ('odom', '/vins_estimator/imu_propagate'),
            ('imu', '/mavros/imu/data_raw'),
            ('position_cmd', '/so3_control/pos_cmd'),
            ('so3_cmd', 'so3_cmd')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(hover_thrust_arg)
    
    # Add the nodes
    ld.add_action(network_controller_node)

    return ld
