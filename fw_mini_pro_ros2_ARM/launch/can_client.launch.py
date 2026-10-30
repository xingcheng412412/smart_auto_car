from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('can_iface', default_value='can0'),
        DeclareLaunchArgument('update_rate', default_value='20.0'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.25'),
        DeclareLaunchArgument('max_linear_speed', default_value='1.5'),
        DeclareLaunchArgument('max_angular_speed', default_value='3.0'),
    ]

    vehicle_core = Node(
        package='can_client',
        executable='vehicle_core',
        name='vehicle_core',
        output='screen',
        parameters=[{
            'can_iface': LaunchConfiguration('can_iface'),
            'command_timeout_sec': ParameterValue(
                LaunchConfiguration('command_timeout_sec'), value_type=float),
            'update_rate': ParameterValue(
                LaunchConfiguration('update_rate'), value_type=float),
            'max_linear_speed': ParameterValue(
                LaunchConfiguration('max_linear_speed'), value_type=float),
            'max_angular_speed': ParameterValue(
                LaunchConfiguration('max_angular_speed'), value_type=float),
        }],
    )

    return LaunchDescription(arguments + [vehicle_core])
