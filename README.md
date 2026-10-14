# YESLab-Gazebo-VINS-PX4
Ubuntu 20.04 + ROS Noetic + PX4 SITL + Gazebo + VINS-Fusion 无GPS Position(POSCTL)模式仿真复现
# YESLab-Gazebo-VINS-PX4

基于 Ubuntu 20.04、ROS Noetic、PX4 SITL、Gazebo、MAVROS 和 VINS-Fusion 实现的无 GPS 四旋翼 Position（POSCTL）模式仿真复现。

本项目完成的核心链路为：

```text
Gazebo Camera + IMU
        ↓
    VINS-Fusion
        ↓
 Pose / Odometry
        ↓
      MAVROS
        ↓
 PX4 External Vision
        ↓
      PX4 EKF2
        ↓
   Local Position
        ↓
 Position / POSCTL
```

最终目标是在 GPS 不参与位置估计的情况下，由 VINS-Fusion 为 PX4 提供外部视觉定位，使无人机能够进入 Position（POSCTL）模式，并完成起飞、悬停、前后左右移动、升降和自动降落。

---

## 1. 实验环境

本次实际复现环境如下：

| 软件 / 环境 | 实际版本 |
|---|---|
| Ubuntu | 20.04.6 LTS |
| ROS | Noetic |
| MAVROS | 1.20.1 |
| Gazebo | 11.15.1 |
| PX4 | v1.14.3 |
| PX4 Commit | `1dacb4c` |
| VINS-Fusion Branch | `master` |
| VINS-Fusion Commit | `be55a93` |
| 仿真无人机 | iris |

PX4 源码路径：

```text
/home/noetic/PX4-v1.14.3
```

VINS-Fusion 源码路径：

```text
/home/noetic/vins_ws/src/VINS-Fusion
```

---

## 2. 项目目录

```text
YESLab-Gazebo-VINS-PX4/
├── README.md
├── config/
│   └── vins_config.yaml
├── launch/
│   └── px4_vins_sim_clock.launch
├── scripts/
│   ├── vins_to_vision_pose.py
│   └── posctl_first_hover_test.py
└── images/
    ├── gazebo.png
    ├── vins_rviz.png
    └── px4_local_position.png
```

其中：

- `config/vins_config.yaml`：Gazebo 双目相机 + IMU 的 VINS-Fusion 配置文件。
- `launch/px4_vins_sim_clock.launch`：PX4、Gazebo、MAVROS 仿真启动相关文件。
- `scripts/vins_to_vision_pose.py`：将 VINS 位姿发送到 MAVROS External Vision。
- `scripts/posctl_first_hover_test.py`：POSCTL 起飞、悬停、移动、升降和降落测试程序。
- `images/`：实验结果截图。

---

## 3. 系统数据流

Gazebo 负责提供仿真无人机、相机和 IMU 数据。

VINS-Fusion 使用视觉和惯性数据估计无人机的位置和姿态。

VINS 的定位结果经过转发后发送至 MAVROS External Vision 接口，再进入 PX4 EKF2。

PX4 EKF2 融合 External Vision 数据并生成 Local Position，最后由 Position（POSCTL）模式完成位置控制。

完整链路如下：

```text
Gazebo iris
     │
     ├── Stereo Camera
     │       ↓
     │ /stereo/left/image_raw
     │
     └── IMU
             ↓
       /vins/imu_gazebo
             ↓
        VINS-Fusion
             ↓
 /vins_estimator/odometry
 /vins_estimator/imu_propagate
             ↓
  vins_to_vision_pose.py
             ↓
 /mavros/vision_pose/pose
             ↓
          MAVROS
             ↓
          MAVLink
             ↓
          PX4 EKF2
             ↓
 /mavros/local_position/pose
             ↓
           POSCTL
```

VINS-Fusion 负责：

```text
根据 Camera + IMU 数据估计无人机在哪里
```

PX4 负责：

```text
使用该位置估计控制无人机飞行
```

因此 VINS-Fusion 与 PX4 的作用不同。

---

## 4. 实际使用的 ROS Topic

本次实验实际使用的主要 Topic 如下：

| 功能 | Topic |
|---|---|
| Gazebo 左目相机 | `/stereo/left/image_raw` |
| Gazebo IMU | `/vins/imu_gazebo` |
| VINS Odometry | `/vins_estimator/odometry` |
| VINS IMU Propagation | `/vins_estimator/imu_propagate` |
| MAVROS External Vision | `/mavros/vision_pose/pose` |
| PX4 Local Position | `/mavros/local_position/pose` |
| PX4 状态 | `/mavros/state` |
| PX4 着陆状态 | `/mavros/extended_state` |
| MAVROS 手动控制 | `/mavros/manual_control/send` |
| Gazebo Ground Truth，仅用于调试对比 | `/gazebo/model_states` |

实际测试过程中观察到的典型频率约为：

```text
/stereo/left/image_raw        ≈ 6.6 Hz
/vins/imu_gazebo              ≈ 198 ~ 199 Hz
/vins_estimator/odometry      ≈ 3 Hz
/mavros/vision_pose/pose      ≈ 49 ~ 50 Hz
```

其中 `/gazebo/model_states` 只用于调试阶段比较 Gazebo 真值、VINS 和 PX4 的位置与姿态，没有直接作为 PX4 的位置输入。

---

## 5. VINS-Fusion 配置

实际使用的配置文件为：

```text
/home/noetic/vins_ws/src/VINS-Fusion/config/px4_stereo/stereo_imu_gazebo_config.yaml
```

GitHub 仓库中保存为：

```text
config/vins_config.yaml
```

配置过程中主要检查和修改：

- Camera Topic
- IMU Topic
- 双目相机模式
- 相机内参
- 图像分辨率
- Camera 与 IMU 外参关系
- IMU 数据频率
- 图像与 IMU 时间戳

VINS-Fusion 的输入来自 Gazebo 仿真中的相机和 IMU，而不是 EuRoC 等离线公开数据集。

---

## 6. VINS 定位发送给 PX4

VINS 的定位结果不能只用于 RViz 显示，还必须真正发送给 PX4。

本项目使用：

```text
scripts/vins_to_vision_pose.py
```

实现：

```text
VINS 位姿
   ↓
坐标与姿态处理
   ↓
/mavros/vision_pose/pose
   ↓
MAVROS
   ↓
PX4 External Vision
```

实际运行过程中：

```text
/mavros/vision_pose/pose ≈ 49 ~ 50 Hz
```

说明外部视觉位姿能够持续发送给 PX4。

---

## 7. ROS 与 PX4 坐标系

ROS 和 PX4 常用坐标系并不完全一致。

ROS 常见：

```text
ENU / FLU
```

PX4 常见：

```text
NED / FRD
```

因此 VINS 输出不能简单把所有 x、y、z 和姿态数据直接复制给 PX4。

本项目通过 `vins_to_vision_pose.py` 完成相关位姿处理，再发送到：

```text
/mavros/vision_pose/pose
```

最终通过前进、后退、左移、右移、升高和降低测试验证位置方向。

---

## 8. PX4 External Vision 配置

实际读取到的 PX4 参数：

```text
EKF2_EV_CTRL = 3
EKF2_HGT_REF = 3
```

测试过程中 PX4 Local Position 状态有效：

```text
xy_valid: True
z_valid: True
v_xy_valid: True
v_z_valid: True
dead_reckoning: False
```

EKF 状态检查中观察到：

```text
filter_fault_flags: 0
innovation_check_flags: 0
health_flags: 0
timeout_flags: 0
```

说明当时没有检测到明显 EKF 故障。

---

## 9. 无 GPS 验证

最终实际参数：

```text
EKF2_GPS_CTRL = 0
```

因此 GPS 不参与最终位置估计。

本项目最终定位链路为：

```text
Gazebo Camera + IMU
        ↓
    VINS-Fusion
        ↓
 MAVROS External Vision
        ↓
      PX4 EKF2
        ↓
   Local Position
        ↓
      POSCTL
```

Gazebo `/gazebo/model_states` 只在调试阶段用于 Ground Truth 对比，没有直接作为 PX4 的定位输入。

因此最终 Position（POSCTL）模式依赖的是 VINS-Fusion 提供的外部视觉定位，而不是 GPS 或 Gazebo Ground Truth。

---

## 10. 完整启动顺序

### 10.1 启动 PX4 + Gazebo

```bash
roslaunch /home/noetic/px4_vins_sim_clock.launch
```

启动后检查 MAVROS：

```bash
rostopic echo -n 1 /mavros/state
```

正常可以看到：

```text
connected: True
```

---

### 10.2 加载 ROS 和 VINS 环境

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/vins_ws/devel/setup.bash
```

---

### 10.3 启动 VINS-Fusion

```bash
rosrun vins vins_node /home/noetic/vins_ws/src/VINS-Fusion/config/px4_stereo/stereo_imu_gazebo_config.yaml
```

检查 VINS：

```bash
rostopic echo -n 1 /vins_estimator/odometry
```

---

### 10.4 启动 VINS → PX4 外部视觉转发

新终端执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/vins_ws/devel/setup.bash
python3 /home/noetic/vins_to_vision_pose.py
```

检查：

```bash
rostopic hz /mavros/vision_pose/pose
```

本次实验约为：

```text
49 ~ 50 Hz
```

---

### 10.5 启动 RViz

```bash
rviz -d /home/noetic/vins_demo.rviz
```

在 RViz 中可以观察 VINS 连续轨迹。

---

### 10.6 检查 Camera、IMU、VINS 和 External Vision

```bash
timeout 5s rostopic hz \
/stereo/left/image_raw \
/vins/imu_gazebo \
/vins_estimator/odometry \
/mavros/vision_pose/pose
```

---

### 10.7 短悬停测试

```bash
python3 -u /home/noetic/posctl_first_hover_test.py --hover 5
```

---

### 10.8 最终飞行测试

```bash
python3 -u /home/noetic/posctl_first_hover_test.py --hover 30 --motion
```

---

## 11. POSCTL 测试流程

最终测试流程如下：

```text
地面状态检查
↓
切换 POSCTL
↓
正常解锁
↓
起飞约 1 m
↓
稳定
↓
悬停
↓
前进
↓
松杆保持
↓
后退
↓
松杆保持
↓
左移
↓
松杆保持
↓
右移
↓
松杆保持
↓
升高
↓
保持
↓
降低
↓
保持
↓
自动降落
↓
自动上锁
```

短悬停测试中，无人机成功升至约：

```text
1.1 ~ 1.2 m
```

典型悬停数据：

```text
PX4上升 ≈ 1.15 m
VINS上升 ≈ 1.16 m
```

PX4 与 VINS 的高度变化基本一致。

最终降落后可确认：

```text
armed: False
landed_state: 1
```

---

## 12. Gazebo、VINS 与 PX4 对比

问题排查阶段同时读取了：

```text
Gazebo Ground Truth
VINS External Vision
PX4 Local Position
```

无人机静止时连续采样发现 VINS 与 PX4 差值很小，例如：

```text
X 差值约 0.001 ~ 0.003 m
Y 差值约 0.003 ~ 0.006 m
Z 差值约 0.009 ~ 0.010 m
```

说明 PX4 Local Position 能够跟随 VINS External Vision。

Gazebo 与 VINS/PX4 之间存在坐标原点偏移，因此 Gazebo 绝对坐标没有直接作为 VINS 定位误差判断依据。

---

## 13. 实际遇到的问题

### 问题一：正常解锁后无人机无法起飞

最初起飞控制设置：

```python
command['z'] = 650.0
```

测试中虽然已经出现：

```text
armed=True
mode=POSCTL
```

但是：

```text
PX4上升 ≈ 0 m
VINS上升 ≈ 0 m
```

进一步检查：

```text
manual_control_setpoint
valid: True
```

说明 MAVROS → PX4 手动控制链路正常。

同时检查 `actuator_motors`，确认 PX4 已经产生电机控制输出。

最终将起飞控制量修改为：

```python
command['z'] = 750.0
```

修改后无人机能够正常离地并升至约 1 m。

---

### 问题二：VINS 时间戳偶发延迟

测试过程中曾出现：

```text
vins 时间戳异常：数据年龄约 0.16 ~ 0.18 s
```

正常情况下 VINS 数据年龄大多约：

```text
0.01 ~ 0.03 s
```

原测试程序允许的最大延迟：

```text
0.15 s
```

由于仿真运行中偶尔出现短时调度延迟，因此调整为：

```text
0.25 s
```

之后短悬停测试能够正常完成。

---

### 问题三：仿真电池状态停止更新

曾出现：

```text
battery_unhealthy: True
pre_flight_checks_pass: False
```

进一步检查发现 `battery_status` 的时间戳数百秒没有更新。

通过：

```bash
battery_simulator stop
battery_simulator start
```

重新启动仿真电池模块后：

```text
battery_unhealthy: False
```

恢复正常解锁。

---

## 14. 安全保护

测试程序没有使用强制解锁。

正常解锁失败时程序会直接停止：

```text
正常解锁被拒绝，没有使用强制解锁
```

本次测试中设置：

```text
NAV_RCL_ACT = 3
COM_POSCTL_NAVL = 1
```

用于控制输入异常时进入安全降落处理。

程序还会持续监测：

- PX4 状态
- VINS 数据
- External Vision
- Local Position
- 着陆状态
- 控制循环
- 数据时间戳

测试结束后程序自动执行降落处理，并等待：

```text
landed_state = 1
armed = False
```

---

## 15. 结果截图

### Gazebo 仿真环境

![Gazebo](images/gazebo.png)

### VINS RViz 连续轨迹

![VINS RViz](images/vins_rviz.png)

### PX4 Local Position

![PX4 Local Position](images/px4_local_position.png)

---

## 16. 项目总结

通过本项目，我完成了以下完整链路：

```text
传感器
↓
状态估计
↓
飞控融合
↓
位置控制
```

Gazebo 负责提供仿真无人机、Camera 和 IMU 数据。

VINS-Fusion 根据视觉和惯性数据估计无人机的位置和姿态。

MAVROS 负责 ROS 与 PX4/MAVLink 之间的数据通信。

PX4 EKF2 融合 VINS-Fusion 提供的 External Vision 数据，并获得 Local Position。

最后 PX4 使用 Local Position 完成 Position（POSCTL）模式的位置控制。

因此，即使 GPS 不参与位置估计，只要 VINS-Fusion 能够持续提供可靠的外部视觉定位，PX4 仍然可以获得有效 Local Position，并完成悬停、位置移动、高度变化和降落。

---

## 17. 参考项目及本人实际修改内容

本项目使用了以下已有开源项目：

- PX4-Autopilot
- ROS Noetic
- MAVROS
- Gazebo
- VINS-Fusion

本人主要完成和调试的内容包括：

- Gazebo Camera / IMU 接入 VINS-Fusion
- VINS Gazebo 配置
- VINS → MAVROS External Vision 转发
- ROS / PX4 坐标与姿态链路验证
- PX4 EKF2 External Vision 配置与检查
- 无 GPS Position 模式验证
- POSCTL 起飞与悬停测试
- 前后左右位置移动测试
- 高度变化测试
- VINS 时间戳问题排查
- 仿真电池异常排查
- 最终演示录屏

---

## 18. 最终完成状态

目前已经完成：

```text
PX4 SITL + Gazebo            ✓
Gazebo Camera                ✓
Gazebo IMU                   ✓
VINS-Fusion                  ✓
VINS → MAVROS                ✓
MAVROS → PX4 External Vision ✓
PX4 Local Position           ✓
GPS 融合关闭                  ✓
POSCTL                       ✓
起飞                          ✓
悬停                          ✓
前后左右移动                  ✓
高度变化                      ✓
自动降落                      ✓
自动上锁                      ✓
演示录屏                      ✓
```

最终完整链路：

```text
Gazebo Camera + IMU
        ↓
    VINS-Fusion
        ↓
 MAVROS External Vision
        ↓
      PX4 EKF2
        ↓
   Local Position
        ↓
      POSCTL
```
