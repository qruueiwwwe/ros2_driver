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
    
    # 添加中继节点的参数
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum linear speed for the relay node'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.5',
        description='Maximum angular speed for the relay node'
    )
    
    safety_check_arg = DeclareLaunchArgument(
        'safety_check',
        default_value='true',
        description='Enable safety checks in the relay node'
    )

    # 创建驱动节点
    driver_node = Node(
        package='ros2_driver',
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
    
    # 创建中继节点
    relay_node = Node(
        package='ros2_driver',
        executable='cmd_vel_relay_node',
        name='cmd_vel_relay',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'acceleration': 0.1,
            'deceleration': 0.2,
            'safety_check': LaunchConfiguration('safety_check')
        }],
        output='screen'
    )

    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,
        http_port_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        safety_check_arg,
        driver_node,
        relay_node
    ]) 