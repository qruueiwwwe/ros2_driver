import unittest
import rclpy
from rclpy.node import Node
from demo.driver_node import DeviceShifuDriver
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class TestDeviceShifuDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DeviceShifuDriver()
        
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        
    def setUp(self):
        self.received_messages = []
        
    def test_initialization(self):
        """测试驱动初始化"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.linear_speed, 0.5)
        self.assertEqual(self.node.angular_speed, 0.2)
        self.assertEqual(self.node.http_port, 5000)
        
    def test_movement_commands(self):
        """测试运动命令"""
        # 创建订阅者来接收命令
        self.cmd_vel_received = False
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 测试前进命令
        self.node.current_command = 1
        self.node.is_moving = True
        self.node.movement_callback()
        
        # 等待消息接收
        time.sleep(0.1)
        
        self.assertTrue(self.cmd_vel_received)
        self.assertEqual(self.received_messages[-1].linear.x, 0.5)
        self.assertEqual(self.received_messages[-1].angular.z, 0.0)
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel_received = True
        self.received_messages.append(msg)
        
    def test_safety_checks(self):
        """测试安全检查"""
        # 测试速度限制
        self.node.current_linear_speed = 2.0
        self.assertFalse(self.node.check_safety())
        
        self.node.current_linear_speed = 0.5
        self.assertTrue(self.node.check_safety())

if __name__ == '__main__':
    unittest.main() 