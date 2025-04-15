from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Linear speed limit'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.2',
        description='Angular speed limit'
    )
    
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='5000',
        description='HTTP server port'
    )
    
    # 创建节点
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
    
    cmd_vel_relay_node = Node(
        package='ros2_driver',
        executable='cmd_vel_relay_node',
        name='cmd_vel_relay',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('linear_speed'),
            'max_angular_speed': LaunchConfiguration('angular_speed'),
            'acceleration': 0.1,
            'deceleration': 0.2,
            'safety_check': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,
        http_port_arg,
        driver_node,
        cmd_vel_relay_node
    ]) 