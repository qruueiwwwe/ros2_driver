from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Maximum linear speed of the robot'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.2',
        description='Maximum angular speed of the robot'
    )
    
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='5000',
        description='HTTP server port'
    )

    # 创建节点
    driver_node = Node(
        package='ros2',
        executable='driver_node',
        name='deviceshifu_driver',
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'http_port': LaunchConfiguration('http_port'),
            'acceleration': 0.1,
            'deceleration': 0.2
        }],
        output='screen'
    )

    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,
        http_port_arg,
        driver_node
    ]) 