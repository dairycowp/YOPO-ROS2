from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    init_x = LaunchConfiguration('init_x', default='0.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_z = LaunchConfiguration('init_z', default='2.0')

    # Declare launch arguments
    init_x_arg = DeclareLaunchArgument(
        'init_x',
        default_value='0.0',
        description='Initial x position'
    )
    
    init_y_arg = DeclareLaunchArgument(
        'init_y',
        default_value='0.0',
        description='Initial y position'
    )
    
    init_z_arg = DeclareLaunchArgument(
        'init_z',
        default_value='2.0',
        description='Initial z position'
    )

    # Quadrotor simulator node
    quadrotor_simulator_node = Node(
        package='so3_quadrotor_simulator',
        executable='quadrotor_simulator_so3',
        name='quadrotor_simulator_so3',
        output='screen',
        parameters=[
            {
                'rate.odom': 50.0,
                'simulator.init_state_x': init_x,
                'simulator.init_state_y': init_y,
                'simulator.init_state_z': init_z
            }
        ],
        remappings=[
            ('odom', '/sim/odom'),
            ('imu', '/sim/imu'),
            ('cmd', 'so3_cmd'),
            ('force_disturbance', 'force_disturbance'),
            ('moment_disturbance', 'moment_disturbance')
        ]
    )

    # Network controller node
    network_controller_node = Node(
        package='so3_control',
        executable='network_control_node',
        name='network_controller_node',
        output='screen',
        parameters=[
            {
                'is_simulation': True,
                'use_disturbance_observer': True,
                'hover_thrust': 0.375,
                'record_log': False,
                'logger_file_name': '/home/hu/YOPO_backup/YOPO/Controller/src/so3_control/logger/'
            }
        ],
        remappings=[
            ('odom', '/sim/odom'),
            ('imu', '/sim/imu'),
            ('position_cmd', '/so3_control/pos_cmd'),
            ('so3_cmd', 'so3_cmd')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(init_x_arg)
    ld.add_action(init_y_arg)
    ld.add_action(init_z_arg)
    
    # Add the nodes
    ld.add_action(quadrotor_simulator_node)
    ld.add_action(network_controller_node)

    return ld