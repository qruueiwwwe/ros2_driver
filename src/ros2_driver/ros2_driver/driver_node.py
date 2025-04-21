#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu
import time
from flask import Flask, request, jsonify
import threading
import os
import socket
import json


class DeviceShifuDriver(Node):
    def __init__(self):
        super().__init__('deviceshifu_driver')

        # Initialize parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('http_port', 5000)
        self.declare_parameter('acceleration', 0.1)  # 加速度限制
        self.declare_parameter('deceleration', 0.2)  # 减速度限制
        
        # 添加速度限制
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_angular_speed', 0.1)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.http_port = self.get_parameter('http_port').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value

        self.get_logger().info(f'Initialized with speeds - Linear: {self.linear_speed}, Angular: {self.angular_speed}')

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/custom_cmd_vel', 10)
        self.get_logger().info('Created publisher for /custom_cmd_vel')

        # Initialize movement state
        self.current_command = 0  # 0: stop, 1: forward, 2: backward, 3: left, 4: right
        self.is_moving = False
        self.movement_timer = None
        
        # 初始化传感器数据
        self.sensor_data = {
            'imu': None,
            'power_voltage': None,
            'imu_low': None
        }
        
        # 创建传感器数据订阅
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu', 
            self.imu_callback, 
            10
        )
        self.power_voltage_sub = self.create_subscription(
            Float32, 
            '/PowerVoltage', 
            self.power_voltage_callback, 
            10
        )
        self.get_logger().info('已创建电源电压订阅，话题: /PowerVoltage')
        self.imu_low_sub = self.create_subscription(
            Imu, 
            '/imu_low', 
            self.imu_low_callback, 
            10
        )

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
        
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.sensor_data['imu'] = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        
    def imu_low_callback(self, msg):
        """低频率IMU数据回调"""
        self.sensor_data['imu_low'] = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        
    def power_voltage_callback(self, msg):
        """电源电压数据回调"""
        self.sensor_data['power_voltage'] = msg.data
        self.get_logger().debug(f'收到电源电压数据: {msg.data}V')

    def setup_routes(self):
        @self.app.route('/move', methods=['POST'])
        def move():
            try:
                # 支持URL参数和JSON格式
                if request.is_json:
                    data = request.get_json()
                    command = data.get('command')
                    linear_speed = data.get('linear_speed')
                    angular_speed = data.get('angular_speed')
                    speed = data.get('speed', 1.0)
                else:
                    # 从URL参数获取数据
                    command = request.args.get('command')
                    linear_speed = request.args.get('linear')
                    angular_speed = request.args.get('angular')
                    speed = float(request.args.get('speed', 1.0))

                if command is not None:
                    self.current_command = int(command)
                    self.is_moving = True
                    
                    # 如果提供了速度参数，更新速度
                    if linear_speed is not None:
                        self.linear_speed = max(min(float(linear_speed), self.max_linear_speed), self.min_linear_speed)
                    if angular_speed is not None:
                        self.angular_speed = max(min(float(angular_speed), self.max_angular_speed), self.min_angular_speed)
                    
                    # 根据command和speed参数调整目标速度
                    if self.current_command == 0:  # 停止命令
                        self.target_linear_speed = 0.0
                        self.target_angular_speed = 0.0
                        self.is_moving = False
                    else:
                        self.target_linear_speed = self.linear_speed * speed
                        self.target_angular_speed = self.angular_speed * speed

                    if self.movement_timer:
                        self.movement_timer.cancel()

                    if self.is_moving:
                        self.movement_timer = self.create_timer(0.1, self.movement_callback)
                        self.get_logger().info(f'Created movement timer for command {command}')
                    else:
                        # 直接发布停止命令
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd)
                        self.get_logger().info('已发布停止命令')

                    return jsonify({
                        'status': 'success', 
                        'command': command, 
                        'speed': speed,
                        'current_speeds': {
                            'linear': self.target_linear_speed,
                            'angular': self.target_angular_speed
                        }
                    })
                else:
                    return jsonify({'status': 'error', 'message': 'No command provided'}), 400
            except Exception as e:
                self.get_logger().error(f'Move command error: {str(e)}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
                
        @self.app.route('/speed', methods=['POST'])
        def set_speed():
            try:
                # 支持URL参数和JSON格式
                if request.is_json:
                    data = request.get_json()
                    linear_speed = data.get('linear_speed')
                    angular_speed = data.get('angular_speed')
                else:
                    # 从URL参数获取数据
                    linear_speed = request.args.get('linear')
                    angular_speed = request.args.get('angular')
                
                speed_changed = False
                
                if linear_speed is not None:
                    new_linear = max(min(float(linear_speed), self.max_linear_speed), self.min_linear_speed)
                    if new_linear != self.linear_speed:
                        self.linear_speed = new_linear
                        speed_changed = True
                        
                if angular_speed is not None:
                    new_angular = max(min(float(angular_speed), self.max_angular_speed), self.min_angular_speed)
                    if new_angular != self.angular_speed:
                        self.angular_speed = new_angular
                        speed_changed = True
                
                # 如果速度发生变化且小车正在移动，更新目标速度
                if speed_changed and self.is_moving:
                    if self.current_command == 1:  # 前进
                        self.target_linear_speed = self.linear_speed
                        self.target_angular_speed = 0.0
                    elif self.current_command == 2:  # 后退
                        self.target_linear_speed = -self.linear_speed
                        self.target_angular_speed = 0.0
                    elif self.current_command == 3:  # 左转
                        self.target_linear_speed = 0.0
                        self.target_angular_speed = self.angular_speed
                    elif self.current_command == 4:  # 右转
                        self.target_linear_speed = 0.0
                        self.target_angular_speed = -self.angular_speed
                
                return jsonify({
                    'status': 'success',
                    'current_speeds': {
                        'linear': self.target_linear_speed,
                        'angular': self.target_angular_speed
                    },
                    'base_speeds': {
                        'linear': self.linear_speed,
                        'angular': self.angular_speed
                    }
                })
            except Exception as e:
                self.get_logger().error(f'Set speed error: {str(e)}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
                
        @self.app.route('/speed', methods=['GET'])
        def get_speed():
            return jsonify({
                'status': 'success',
                'current_speeds': {
                    'linear': self.target_linear_speed,
                    'angular': self.target_angular_speed
                },
                'base_speeds': {
                    'linear': self.linear_speed,
                    'angular': self.angular_speed
                },
                'speed_limits': {
                    'linear': {
                        'min': self.min_linear_speed,
                        'max': self.max_linear_speed
                    },
                    'angular': {
                        'min': self.min_angular_speed,
                        'max': self.max_angular_speed
                    }
                }
            })

        @self.app.route('/stop', methods=['POST'])
        def stop():
            try:
                self.get_logger().info('收到停止命令')
                self.current_command = 0
                self.is_moving = False
                
                # 直接发布停止命令
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info('已发布停止命令')
                
                # 重置目标速度
                self.target_linear_speed = 0.0
                self.target_angular_speed = 0.0
                
                if self.movement_timer:
                    self.movement_timer.cancel()
                    self.get_logger().info('已取消移动定时器')
                
                return jsonify({'status': 'success', 'message': 'Robot stopped'})
            except Exception as e:
                self.get_logger().error(f'Stop command error: {str(e)}')
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/status', methods=['GET'])
        def status():
            return jsonify({
                'status': 'success',
                'is_moving': self.is_moving,
                'current_command': self.current_command,
                'linear_speed': self.linear_speed,
                'angular_speed': self.angular_speed,
                'sensor_data': self.sensor_data
            })
            
        @self.app.route('/sensors', methods=['GET'])
        def sensors():
            """获取所有传感器数据"""
            return jsonify({
                'status': 'success',
                'sensor_data': self.sensor_data
            })
            
        @self.app.route('/imu', methods=['GET'])
        def imu():
            """获取IMU数据"""
            return jsonify({
                'status': 'success',
                'imu_data': self.sensor_data['imu']
            })
            
        @self.app.route('/power_voltage', methods=['GET'])
        def power_voltage():
            """获取电源电压数据"""
            return jsonify({
                'status': 'success',
                'voltage': self.sensor_data['power_voltage']
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
        if self.current_command == 0:  # Stop
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
        self.get_logger().info(f'Published command - linear: {self.current_linear_speed}, angular: {self.current_angular_speed}')

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