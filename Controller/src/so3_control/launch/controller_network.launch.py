from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    hover_thrust = LaunchConfiguration('hover_thrust', default='0.38')

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
                'logger_file_name': '/home/hu/YOPO_backup/YOPO/Controller/src/so3_control/logger/'
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