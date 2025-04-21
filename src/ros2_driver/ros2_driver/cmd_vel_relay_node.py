#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.get_logger().info('CmdVelRelayNode initialized')

        # 声明参数
        self.declare_parameter('max_linear_speed', 1.0)  # 最大线速度
        self.declare_parameter('max_angular_speed', 0.5)  # 最大角速度
        self.declare_parameter('acceleration', 0.1)  # 加速度限制
        self.declare_parameter('deceleration', 0.2)  # 减速度限制
        self.declare_parameter('safety_check', True)  # 是否启用安全检查
        self.declare_parameter('min_linear_speed', 0.1)  # 最小线速度
        self.declare_parameter('min_angular_speed', 0.1)  # 最小角速度

        # 获取参数值
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.acceleration = self.get_parameter('acceleration').value
        self.deceleration = self.get_parameter('deceleration').value
        self.safety_check = self.get_parameter('safety_check').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value

        self.get_logger().info(
            f'初始化中继节点 - 最大线速度: {self.max_linear_speed}, 最大角速度: {self.max_angular_speed}')

        # 创建订阅者和发布者
        self.custom_cmd_vel_sub = self.create_subscription(
            Twist,
            '/custom_cmd_vel',
            self.custom_cmd_vel_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化速度控制相关变量
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.target_linear_speed = 0.0
        self.target_angular_speed = 0.0
        self.last_command_time = self.get_clock().now()

        # 创建定时器，用于平滑速度变化
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('命令速度中继节点已初始化')

    def custom_cmd_vel_callback(self, msg):
        """处理来自 /custom_cmd_vel 的消息"""
        # 设置目标速度
        self.target_linear_speed = msg.linear.x
        self.target_angular_speed = msg.angular.z
        
        # 更新最后命令时间
        this_time = self.get_clock().now()
        time_diff = (this_time - self.last_command_time).nanoseconds / 1e9
        self.last_command_time = this_time
        
        # 记录接收到的命令
        self.get_logger().debug(
            f'接收到命令 - 线速度: {self.target_linear_speed}, 角速度: {self.target_angular_speed}, 时间间隔: {time_diff:.3f}s')

    def timer_callback(self):
        """定时器回调，用于平滑速度变化并发布到 /cmd_vel"""
        # 平滑速度变化
        self.current_linear_speed = self.smooth_speed_change(
            self.current_linear_speed,
            self.target_linear_speed,
            self.acceleration,
            self.deceleration
        )

        self.current_angular_speed = self.smooth_speed_change(
            self.current_angular_speed,
            self.target_angular_speed,
            self.acceleration,
            self.deceleration
        )

        # 安全检查
        if self.safety_check and not self.check_safety():
            self.get_logger().warn('安全检查失败，停止小车')
            self.current_linear_speed = 0.0
            self.current_angular_speed = 0.0

        # 创建并发布 Twist 消息
        cmd = Twist()
        cmd.linear.x = self.current_linear_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.current_angular_speed

        # 发布命令
        self.cmd_vel_pub.publish(cmd)
        
        # 记录当前速度
        if abs(self.current_linear_speed) > 0.01 or abs(self.current_angular_speed) > 0.01:
            self.get_logger().debug(
                f'当前速度 - 线速度: {self.current_linear_speed:.3f}, 角速度: {self.current_angular_speed:.3f}')

    def smooth_speed_change(self, current, target, accel, decel):
        """平滑速度变化"""
        if abs(target - current) < 0.01:
            return target

        if target > current:
            return min(current + accel, target)
        else:
            return max(current - decel, target)

    def check_safety(self):
        """安全检查"""
        # 检查速度是否在安全范围内
        if abs(self.current_linear_speed) > self.max_linear_speed:
            self.get_logger().warn('线速度超过安全限制')
            return False

        if abs(self.current_angular_speed) > self.max_angular_speed:
            self.get_logger().warn('角速度超过安全限制')
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    relay_node = CmdVelRelayNode()
    try:
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        relay_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()