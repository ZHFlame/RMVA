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

### 🎯 弹丸击打与弹道预测
- EKF 圆轨迹估计：圆心、半径、角速度
- PnP 位姿求解
- 基于飞行时间的未来击打点预测
- 欧拉角（yaw/pitch/roll）返回服务端

### 🛰 ROS2 原生集成
- rclcpp 节点
- 物体消息 MultiObject
- 裁判系统信息订阅

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


## 3. 核心算法详解

### 🎯 弹道预测与击打策略

本系统采用 **PnP 解算 → EKF 预测 → 弹道补偿** 的闭环链路，确保在目标移动和重力影响下仍能精准命中。

---

## 1. 坐标系定义

**相机坐标系（OpenCV 标准）**  
- X：向右  
- Y：向下  
- Z：向前（深度方向）

**物理世界坐标系（弹道模型）**  
- Y 轴取反，使 **向上为正**，符合物理抛物线习惯。

---

## 2. PnP 位姿解算（`transform_2d_3d`）

- 算法：`cv::solvePnP`（`SOLVEPNP_IPPE` 稳定平面目标）
- 装甲板模型尺寸：长 **0.705 m**，高 **0.230 m**
- 点序：**左下 → 右下 → 右上 → 左上**（逆时针）
- 输出：装甲板中心坐标 **(x, y, z)**（相机坐标系）

---

## 3. EKF 圆周运动预测（`CorrectCircularMotionEKF`）

目标存在周期性旋转运动，使用 EKF 对其进行估计和预测。

### 状态向量

$$
X = [x,\; z,\; v_x,\; v_z,\; \omega]^T
$$

- **$(x, z)$**：XZ 平面（水平面）内的位置；
- **$(v_x, v_z)$**：水平速度；
- **$\omega$**：旋转角速度。

---

### 飞行时间计算

子弹飞行时间：

$$
t_{fly} = \frac{\sqrt{x^2 + y^2 + z^2}}{v}
$$

其中 **$(v)$** 为弹速（默认 15 m/s）。

---

### 圆周预测结果

根据角速度 **$\omega$** 推算未来位置 **$(x_{pred}, z_{pred})$**。

---

## 4. 弹道补偿（`calculate`）

### 4.1 Yaw（偏航角）

$$
\psi = \tan^{-1}\left(\frac{x_{pred}}{z_{pred}}\right)
$$

---

### 4.2 Pitch（俯仰角）

基于抛物线标准模型：

$$
y_{target} = d \cdot \tan\theta \;-\; \frac{g d^2}{2 v^2 \cos^2\theta}
$$

其中： 
- **$d = \sqrt{x^2 + z^2}$**：水平距离； 
- **$y_{target}$**：目标高度（取反后的物理高度）； 
- **$v$**：当前弹速（默认为 **15 m/s**，需按实际调整）；
- **$g$**：重力加速度。

---

**求解策略**： 
- 若方程有解（判别式 **$\Delta \ge 0$**）：取较小的 **$\theta$**（直射弹道，飞行时间短）；
- 若方程无解（超出射程）：退化为直接指向目标：

$$
\theta = \tan^{-1}\left(\frac{y_{target}}{d}\right)
$$

---


### 👁️ 图像预处理与多目标检测 (`detector.cpp`)

为了适应复杂光照环境并同时检测装甲板、球体和实心矩形，系统采用了多通道融合的边缘检测方案。

#### 1. 多通道边缘融合
传统的 Canny 边缘检测在单一灰度图上容易丢失特定颜色的边缘。本方案分别在 **B、G、R 三个通道** 以及 **灰度通道** 上独立进行 Canny 检测，然后取并集。
*   **流程**：
    1.  `split(src, channels)` 分离 BGR 通道。
    2.  对每个通道进行 `medianBlur` 去噪 + `Canny` 边缘提取。
    3.  `bitwise_or` 融合所有通道的边缘。
    4.  `morphologyEx` (闭运算) 连接断裂边缘，`dilate` (膨胀) 填充空洞。
*   **优势**：能有效捕获色彩对比度低但亮度差异大的边缘，也能捕获亮度相近但色相差异大的边缘。

#### 2. 颜色分类策略
针对非装甲板目标（球体、方块），系统基于 ROI 的平均 BGR 值进行决策树分类。
*   **亮度判断**：通过 `max(R,G,B) - min(R,G,B)` 判断是否为黑/白/灰。
*   **色相判断**：
    *   **红色系**：R 分量显著高于 G、B，或 R 占比 > 60%。
    *   **蓝色系**：B 分量主导。
    *   **绿色系**：G 分量主导。
    *   **混合色**：针对紫色、橙色、黄色等，设定了特定的 RGB 阈值区间。

---

### 🛡️ 装甲板识别与数字解码 (`armor.cpp`)

装甲板识别是视觉系统的核心，采用 **HSV 颜色分割 -> 椭圆拟合 -> 透视变换 -> 模板匹配** 的流水线。

#### 1. 灯条提取 (`Ellipse`)
*   **颜色分割**：在 HSV 空间使用双阈值提取红色区域（处理红色在 Hue 环上跨越 0 度的问题）。
*   **轮廓筛选**：拟合 `RotatedRect`，根据长宽比和面积筛选潜在灯条。
*   **顶点定位**：
    *   通过比较旋转矩形相邻边的长度，确定长轴方向。
    *   取长轴两端的中点作为灯条的 **上顶点** 和 **下顶点**。
*   **配对逻辑**：
    *   对所有灯条按 Y 轴排序（分上下），再按 X 轴排序（分左右）。
    *   输出顺序：**左下 -> 右下 -> 右上 -> 左上**（逆时针，适配 PnP）。

#### 2. 装甲板重建
利用两个灯条的 4 个顶点，推算装甲板的 4 个角点。
*   计算左右灯条的中心距 $(dx, dy)$。
*   向外扩展：将灯条顶点沿中心距方向向外延伸，构建完整的装甲板矩形 ROI。

#### 3. 数字识别
*   **透视变换**：使用 `warpPerspective` 将倾斜的装甲板 ROI 矫正为 $255 \times 193$ 的标准正视图。
*   **二值化**：`threshold` 处理，突出数字纹理。
*   **模板匹配**：
    *   预加载 1~5 号数字的标准模板。
    *   使用 `matchTemplate` (TM_CCOEFF_NORMED) 计算匹配度。
    *   取最大匹配分数，若 `max_score > 0.5` 则判定为对应数字，否则丢弃。

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
cd ~/rmva
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
FROM vision-vrena-2025:v0.1.2

# 复制现有构建产物、选手包及结果
COPY ./install /home/va/Vision-Vrena-2025/install
COPY ./src/player_pkg /home/va/Vision-Vrena-2025/src/player_pkg
COPY ./results /home/va/Vision-Vrena-2025/results

# 进入工作空间并安装额外依赖
WORKDIR /home/va/Vision-Vrena-2025
RUN apt-get update && \
    apt-get upgrade -y && \
    mkdir -p src/referee_pkg/results && \
    apt-get install -y \
        ros-humble-xacro \
        ros-humble-gazebo-ros-pkgs && \
    rm -rf /var/lib/apt/lists/*


RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select player_pkg

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]
```

### 8.2 docker-compose.yml
```yaml
version: '3.8'

networks:
  ros2-network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.30.0.0/24

services:
  gazebo:
    image: vision-vrena-2025:v0.1.3
    container_name: gazebo
    hostname: gazebo
    networks:
      ros2-network:
        ipv4_address: 172.30.0.10
    environment:
      - ROS_DOMAIN_ID=55
      - ROS_LOCALHOST_ONLY=0
      - GAZEBO_MASTER_URI=http://localhost:11345
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - XAUTHORITY=/tmp/.docker.xauth
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ${XAUTHORITY}:/tmp/.docker.xauth
    working_dir: /home/va/Vision-Vrena-2025
    command: >
      bash -c "
        cd /home/va/Vision-Vrena-2025 && ls  &&
        source install/setup.bash &&
        export GAZEBO_MODE=headless &&  
        ros2 launch camera_sim_pkg camera.launch.py width:=640 height:=640 fps:=50
      "
    tty: true
    stdin_open: true
    restart: unless-stopped
    privileged: true

  team25_challenge:
    image: vision-vrena-2025:v0.1.3
    container_name: team25_challenge
    hostname: team25_challenge
    networks:
      ros2-network:
        ipv4_address: 172.30.0.12
    environment:
      - ROS_DOMAIN_ID=55
      - ROS_LOCALHOST_ONLY=0
      - GAZEBO_MASTER_URI=http://localhost:11345
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - XAUTHORITY=/tmp/.docker.xauth
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ${XAUTHORITY}:/tmp/.docker.xauth
    working_dir: /home/va/Vision-Vrena-2025
    command: >
      bash -c "
        cd /home/va/Vision-Vrena-2025 && ls &&
        source install/setup.bash &&
        ros2 run team25_challenge vision_node 
      "

    tty: true
    stdin_open: true
    restart: unless-stopped
    depends_on:
      - gazebo

  referee:
    image: vision-vrena-2025:v0.1.3
    container_name: referee
    hostname: referee
    networks:
      ros2-network:
        ipv4_address: 172.30.0.11
    environment:
      - ROS_DOMAIN_ID=55
      - ROS_LOCALHOST_ONLY=0
    working_dir: /home/va/Vision-Vrena-2025
    command: >
      bash -c "
        cd /home/va/Vision-Vrena-2025 && ls && 
        source install/setup.bash &&
        ros2 launch referee_pkg referee_pkg_launch.xml 
        "
    tty: true
    stdin_open: true
    restart: unless-stopped
    depends_on:
      - gazebo

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
ros2 launch team25_challenge vision.launch.py
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
