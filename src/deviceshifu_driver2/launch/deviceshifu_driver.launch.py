from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Linear speed for robot movement'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.2',
        description='Angular speed for robot movement'
    )
    
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='5000',
        description='HTTP server port'
    )
    
    # Create and return launch description
    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,
        http_port_arg,
        
        Node(
            package='deviceshifu_driver2',
            executable='driver_node.py',
            name='deviceshifu_driver',
            output='screen',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'http_port': LaunchConfiguration('http_port')
            }]
        )
    ]) 