from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    quadrotor_name = LaunchConfiguration('quadrotor_name', default='quadrotor')
    mass = LaunchConfiguration('mass', default='0.5')
    use_external_yaw = LaunchConfiguration('use_external_yaw', default='true')

    # Declare launch arguments
    quadrotor_name_arg = DeclareLaunchArgument(
        'quadrotor_name',
        default_value='quadrotor',
        description='Name of the quadrotor'
    )
    
    mass_arg = DeclareLaunchArgument(
        'mass',
        default_value='0.5',
        description='Mass of the quadrotor'
    )
    
    use_external_yaw_arg = DeclareLaunchArgument(
        'use_external_yaw',
        default_value='true',
        description='Use external yaw estimation'
    )

    # SO3 Control node
    so3_control_node = Node(
        package='so3_control',
        executable='so3_control_node',
        name='so3_control_node',
        output='screen',
        parameters=[
            {
                'quadrotor_name': quadrotor_name,
                'mass': mass,
                'use_external_yaw': use_external_yaw,
                'record_log': False,
                'gains.rot.x': 1.5,
                'gains.rot.y': 1.5,
                'gains.rot.z': 1.0,
                'gains.ang.x': 0.13,
                'gains.ang.y': 0.13,
                'gains.ang.z': 0.1,
                'gains.kx.x': 5.7,
                'gains.kx.y': 5.7,
                'gains.kx.z': 6.2,
                'gains.kv.x': 3.4,
                'gains.kv.y': 3.4,
                'gains.kv.z': 4.0,
                'corrections.z': 0.0,
                'corrections.r': 0.0,
                'corrections.p': 0.0,
                'so3_control.init_state_x': 0.0,
                'so3_control.init_state_y': 0.0,
                'so3_control.init_state_z': -10000.0
            }
        ],
        remappings=[
            ('odom', 'odom'),
            ('position_cmd', 'position_cmd'),
            ('motors', 'motors'),
            ('corrections', 'corrections'),
            ('imu', 'imu'),
            ('so3_cmd', 'so3_cmd')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(quadrotor_name_arg)
    ld.add_action(mass_arg)
    ld.add_action(use_external_yaw_arg)
    
    # Add the nodes
    ld.add_action(so3_control_node)

    return ld