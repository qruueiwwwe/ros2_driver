# ROS2 Driver Demo

这是一个ROS2驱动程序示例包，用于机器人控制。

## 功能特点

- 提供HTTP接口控制机器人运动
- 支持基本的运动控制（前进、后退、左转、右转、停止）
- 实现平滑的速度控制
- 支持参数配置

## 安装

1. 确保已安装ROS2环境
2. 克隆此仓库到你的工作空间
3. 编译包：
```bash
colcon build --packages-select demo
```

## 使用方法

1. 启动驱动节点：
```bash
ros2 launch demo driver.launch.py
```

2. 使用HTTP API控制机器人：

- 移动机器人：
```bash
curl -X POST http://localhost:5000/move -H "Content-Type: application/json" -d '{"command": 1, "speed": 1.0}'
```

- 停止机器人：
```bash
curl -X POST http://localhost:5000/stop
```

- 获取状态：
```bash
curl http://localhost:5000/status
```

命令说明：
- command: 0=停止, 1=前进, 2=后退, 3=左转, 4=右转
- speed: 速度倍数（0.0-1.0）

## 参数配置

- linear_speed: 最大线速度（默认：0.5）
- angular_speed: 最大角速度（默认：0.2）
- http_port: HTTP服务器端口（默认：5000）
- acceleration: 加速度限制（默认：0.1）
- deceleration: 减速度限制（默认：0.2）

## 依赖

- ROS2
- Flask
- geometry_msgs
- std_msgs
- diagnostic_msgs

## 许可证

TODO 