# team25_challenge

## 1. 项目简介
本仓库基于 **ROS2 + OpenCV** 实现机甲大师校内赛视觉全链路，包括：

- `vision_node`：多目标检测、装甲识别、颜色分析并发布 `/vision/target`
- `shooter_node`：PnP 解析、EKF 圆轨迹预测、服务 `/referee/hit_arror`
- 适用于裁判系统 `referee_pkg`，包含 Docker 镜像构建方案
  

适用于 **哈尔滨工业大学（威海）Vision Arena 2025 校内赛** 的正式比赛环境。

---

## 2. 功能特性

### 🔍 视觉识别
- 多通道边缘融合
- 球体 / 实心矩形 / 装甲板检测
- HSV 双区间灯条提取
- 装甲板透视变换
- 装甲数字模板匹配（1~5）

### 🎯 弹丸击打
- EKF 圆轨迹估计：圆心、半径、角速度
- PnP 位姿求解
- 基于飞行时间的未来击打点预测
- 欧拉角（yaw/pitch/roll）返回服务端

### 🛰 ROS2 原生集成
- rclcpp 节点
- 物体消息 MultiObject
- 裁判系统 RaceStage 订阅

---

## 3. 目录结构

```
team25_challenge/
├── CMakeLists.txt
├── package.xml
├── link.md
├── Dockerfile
├── docker-compose.yml
├── launch/
│   ├── vision.launch.py
│   └── shooter.launch.py
├── src/
│   ├── vision/
│   │   ├── templates/
│   │   ├── testphotos/
│   │   ├── detector.cpp
│   │   ├── armor.cpp
│   │   ├── sphere.cpp
│   │   └── vision.h
│   ├── vision_node.cpp
│   ├── shooter/
│   │   ├── shooter.h
│   │   └── calculate.cpp
│   └── shooter_node.cpp
├── results/
├── include/
├── models/
│   ├── mnist-8.onnx
│   └── mnist-12.onnx
└── README.md
```

---

## 4. 环境依赖
- Ubuntu 22.04
- ROS2 Humble
- OpenCV ≥ 4.5
- Eigen3
- referee_pkg（build + install）
- cv_bridge / sensor_msgs / geometry_msgs / rclcpp

---

## 5. 编译方式

```bash
cd ~/colcon_ws
colcon build --packages-select challenge
source install/setup.bash
```

---

## 6. 运行方式

### 启动视觉节点
```bash
ros2 launch challenge vision.launch.py
```

### 启动击打节点
```bash
ros2 launch challenge shooter.launch.py
```

---

## 7. 数据管线说明
1. 相机 `/camera/image_raw`
2. `vision_node` → `/vision/target`
3. `shooter_node`：
   - PnP 解算
   - EKF XZ 平面圆拟合
4. `/referee/hit_arror` 服务返回 yaw/pitch/roll

---

## 8. Docker 部署方案


### 8.1 Dockerfile
```dockerfile
FROM ros:humble-ros-base

# 基础依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    libopencv-dev \
    libeigen3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 裁判系统 SDK 及依赖
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /rmva/src
COPY . /rmva/src/challenge

WORKDIR /rmva
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select challenge

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /rmva/install/setup.bash && bash"]
```

### 8.2 docker-compose.yml
```yaml
version: '3'
services:
  challenge:
    build: .
    container_name: team25_challenge
    network_mode: host
    volumes:
      - ./results:/rmva/results
      - /dev:/dev
    environment:
      - DISPLAY=${DISPLAY}
    stdin_open: true
    tty: true
```
### 8.3 构建与运行
```bash
docker-compose build
docker-compose up -d
docker exec -it team25_challenge bash
```
### 8.4 容器内运行程序
```bash
source /opt/ros2/humble/setup.bash
source /rmva/install/setup.bash
ros2 launch challenge vision.launch.py
```
## 9. 常见问题（FAQ）

| 问题 | 原因 | 解决方式 |
| ---- | ---- | -------- |
| 模板匹配失败 | 模板缺失/路径错误 | 确保 `templates/1.png~5.png` 存在且容器内路径正确 |
| 灯条未检测到装甲板 | HSV 不适配现场光照 | 调整 `Armor_Detector()` 中红色阈值 |
| PnP 解算数值跳变大 | 角点顺序混乱 / ROI 噪声大 | 检查 `sortPoints()`、增强预处理滤波 |
| EKF 不收敛或圆拟合半径为 0 | 历史数据太少或噪声过大 | 增加 `xz_points` 缓存数量，平滑输入 |
| 服务 `/referee/hit_arror` 无响应 | shooter_node 未启动或未正确注册服务 | 执行 `ros2 service list` 检查是否存在 |
| 容器内无法显示图像 | 无 X11 转发 | 运行前执行：`xhost +local:docker` |
| 无法访问相机 `/dev/video0` | Docker 未挂载设备 | 在 compose 中添加：`devices: ["/dev/video0:/dev/video0"]` |
| 容器构建失败找不到 referee_pkg | 未复制官方 `build/`、`install/` | 确认 Dockerfile 中 `COPY referee_pkg/...` 路径正确 |
| 画面延迟或卡顿 | 内部处理耗时较长 | 调整 Canny 阈值、减少窗口显示、限制 debug 日志量 |

---
## 10. 调试与排障命令
- 查看相机：`ros2 topic echo /camera/image_raw`
- 检查目标发布：`ros2 topic echo /vision/target`
- 调试 Launch：`ros2 launch challenge shooter.launch.py verbose:=true`
- 录制数据包：`ros2 bag record /camera/image_raw /vision/target`