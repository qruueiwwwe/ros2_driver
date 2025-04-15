# ROS2 Driver

这是一个ROS2驱动程序，用于机器人控制，提供HTTP接口。

## 功能

- 通过HTTP接口控制机器人移动
- 支持前进、后退、左转、右转和停止命令
- 提供速度平滑控制，避免突然加速或减速
- 提供安全检查，防止速度超过安全限制

## 依赖

- ROS2 (推荐使用Humble或更高版本)
- Python 3.8+
- Flask

## 安装

1. 克隆仓库到您的工作空间：

```bash
cd ~/ros2_ws/src
git clone <repository-url> ros2_driver
```

2. 安装依赖：

```bash
pip3 install flask
```

3. 构建工作空间：

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_driver
```

4. 设置环境变量：

```bash
source ~/ros2_ws/install/setup.bash
```

## 使用方法

### 启动节点

```bash
ros2 launch ros2_driver driver.launch.py
```

您可以通过参数自定义行为：

```bash
ros2 launch ros2_driver driver.launch.py linear_speed:=0.3 angular_speed:=0.1 http_port:=8080
```

### HTTP API

驱动程序提供以下HTTP API：

#### 移动命令

```
POST /move
```

请求体：

```json
{
  "command": 1,  // 0: 停止, 1: 前进, 2: 后退, 3: 左转, 4: 右转
  "speed": 1.0   // 速度倍数，默认为1.0
}
```

响应：

```json
{
  "status": "success",
  "command": 1,
  "speed": 1.0
}
```

#### 停止命令

```
POST /stop
```

响应：

```json
{
  "status": "success",
  "message": "Robot stopped"
}
```

#### 状态查询

```
GET /status
```

响应：

```json
{
  "status": "success",
  "is_moving": true,
  "current_command": 1,
  "linear_speed": 0.5,
  "angular_speed": 0.2
}
```

## 参数说明

- `linear_speed`: 最大线速度 (默认: 0.5 m/s)
- `angular_speed`: 最大角速度 (默认: 0.2 rad/s)
- `http_port`: HTTP服务器端口 (默认: 5000)
- `acceleration`: 加速度限制 (默认: 0.1)
- `deceleration`: 减速度限制 (默认: 0.2)

## 构建说明

### 使用colcon构建

1. 确保您已经安装了ROS2和所有依赖项。

2. 在工作空间根目录中运行：

```bash
colcon build --packages-select ros2_driver
```

3. 设置环境变量：

```bash
source install/setup.bash
```

### 运行测试

```bash
colcon test --packages-select ros2_driver
```

### 清理构建

```bash
colcon clean --packages-select ros2_driver
```

## 许可证

Apache License 2.0 