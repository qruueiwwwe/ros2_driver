#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from flask import Flask, request, jsonify
import threading
import os
import socket

class DeviceShifuDriver(Node):
    def __init__(self):
        super().__init__('deviceshifu_driver')
        
        # Initialize parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('http_port', 5000)
        self.declare_parameter('acceleration', 0.1)  # 加速度限制
        self.declare_parameter('deceleration', 0.2)  # 减速度限制
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.http_port = self.get_parameter('http_port').value
        
        self.get_logger().info(f'Initialized with speeds - Linear: {self.linear_speed}, Angular: {self.angular_speed}')
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/custom_cmd_vel', 10)
        
        # Initialize movement state
        self.current_command = 0  # 0: stop, 1: forward, 2: backward, 3: left, 4: right
        self.is_moving = False
        self.movement_timer = None
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Start HTTP server in a separate thread
        self.http_thread = threading.Thread(target=self.run_http_server)
        self.http_thread.daemon = True
        self.http_thread.start()
        
        self.get_logger().info('DeviceShifu driver initialized with HTTP server on port %d' % self.http_port)
        
        # 添加速度控制相关变量
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.target_linear_speed = 0.0
        self.target_angular_speed = 0.0

    def setup_routes(self):
        @self.app.route('/move', methods=['POST'])
        def move():
            try:
                data = request.get_json()
                command = data.get('command')
                speed = data.get('speed', 1.0)  # 添加速度控制
                
                if command is not None:
                    self.current_command = int(command)
                    self.is_moving = True
                    
                    # 根据speed参数调整目标速度
                    self.target_linear_speed = self.linear_speed * speed
                    self.target_angular_speed = self.angular_speed * speed
                    
                    if self.movement_timer:
                        self.movement_timer.cancel()
                    
                    self.movement_timer = self.create_timer(0.1, self.movement_callback)
                    return jsonify({'status': 'success', 'command': command, 'speed': speed})
                else:
                    return jsonify({'status': 'error', 'message': 'No command provided'}), 400
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/stop', methods=['POST'])
        def stop():
            try:
                self.current_command = 0
                self.is_moving = False
                if self.movement_timer:
                    self.movement_timer.cancel()
                return jsonify({'status': 'success', 'message': 'Robot stopped'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/status', methods=['GET'])
        def status():
            return jsonify({
                'status': 'success',
                'is_moving': self.is_moving,
                'current_command': self.current_command,
                'linear_speed': self.linear_speed,
                'angular_speed': self.angular_speed
            })

    def run_http_server(self):
        try:
            # 获取容器IP地址
            container_ip = os.environ.get('POD_IP', '0.0.0.0')
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            
            self.get_logger().info(f'Container IP: {container_ip}')
            self.get_logger().info(f'Hostname: {hostname}')
            self.get_logger().info(f'Local IP: {local_ip}')
            self.get_logger().info(f'Starting HTTP server on 0.0.0.0:{self.http_port}')
            
            # 使用0.0.0.0确保服务器监听所有网络接口
            self.app.run(host='0.0.0.0', port=self.http_port, debug=True, use_reloader=False)
        except Exception as e:
            self.get_logger().error(f'Failed to start HTTP server: {str(e)}')

    def movement_callback(self):
        """Handle continuous movement with smooth acceleration"""
        if not self.is_moving:
            return
            
        cmd = Twist()
        
        # 计算目标速度
        if self.current_command == 0:    # Stop
            self.target_linear_speed = 0.0
            self.target_angular_speed = 0.0
        elif self.current_command == 1:  # Forward
            self.target_linear_speed = self.linear_speed
            self.target_angular_speed = 0.0
        elif self.current_command == 2:  # Backward
            self.target_linear_speed = -self.linear_speed
            self.target_angular_speed = 0.0
        elif self.current_command == 3:  # Turn left
            self.target_linear_speed = 0.0
            self.target_angular_speed = self.angular_speed
        elif self.current_command == 4:  # Turn right
            self.target_linear_speed = 0.0
            self.target_angular_speed = -self.angular_speed
        
        # 平滑速度变化
        self.current_linear_speed = self.smooth_speed_change(
            self.current_linear_speed,
            self.target_linear_speed,
            self.get_parameter('acceleration').value,
            self.get_parameter('deceleration').value
        )
        
        self.current_angular_speed = self.smooth_speed_change(
            self.current_angular_speed,
            self.target_angular_speed,
            self.get_parameter('acceleration').value,
            self.get_parameter('deceleration').value
        )
        
        # 设置命令
        cmd.linear.x = self.current_linear_speed
        cmd.angular.z = self.current_angular_speed
        
        # 发布命令
        self.cmd_vel_pub.publish(cmd)
        
        # 检查是否达到目标速度
        if abs(self.current_linear_speed - self.target_linear_speed) < 0.01 and \
           abs(self.current_angular_speed - self.target_angular_speed) < 0.01:
            self.get_logger().debug('Reached target speed')

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
        if abs(self.current_linear_speed) > self.linear_speed * 1.1:
            self.get_logger().warn('Linear speed exceeded safety limit')
            return False
        
        if abs(self.current_angular_speed) > self.angular_speed * 1.1:
            self.get_logger().warn('Angular speed exceeded safety limit')
            return False
        
        return True

def main(args=None):
    rclpy.init(args=args)
    driver = DeviceShifuDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 