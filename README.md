# lift_controller

基于 ROS2 的升降电机控制包，通过 CANopen (CiA 402) 协议控制 DS/RS 系列伺服/步进驱动器，支持 200Hz 高频 PDO 实时控制。

## 功能特性

- **PDO 高频控制**：CSP/PP 模式 + RPDO1 + TPDO1 + SYNC，控制频率可达 200Hz
- **SDO 回退模式**：PDO 配置失败时自动回退到 SDO 模式（~10Hz），保证可用性
- **队列式 CAN 接收**：SDO 响应入队 + condition_variable 等待，避免竞态条件
- **断电重连**：电机掉电后自动检测并重连
- **余弦插值轨迹**：S 型加减速，平滑运动
- **全部参数可配置**：通过 YAML 配置文件加载

## 硬件要求

- CAN 接口（SocketCAN，如 `can0`）
- DS/RS 系列驱动器（CiA 402 兼容）
- 驱动器已配置为 CANopen 模式（`F0.1.002 = 20`）
- 总线两端 120Ω 终端电阻

## 编译

```bash
cd ~/workspace/ros2_ws
colcon build --packages-select lift_controller
source install/setup.bash
```

## CAN 接口配置

启动节点前需配置 CAN 接口（电机默认 1 Mbps）：

```bash
# 方式一：使用自带脚本（推荐）
sudo bash ~/workspace/ros2_ws/src/lift_controller/scripts/setup_can.sh

# 方式二：手动配置
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.875
sudo ip link set can0 up
```

## 启动

```bash
ros2 launch lift_controller lift_motor.launch.py
```

## 配置参数

配置文件位于 [config/lift_config.yaml](config/lift_config.yaml)：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `can_interface` | string | `"can0"` | SocketCAN 接口名 |
| `can_bitrate` | int | `1000000` | CAN 波特率（提示用，实际由 `ip link` 设置） |
| `node_id` | int | `1` | 驱动器 CANopen 节点 ID |
| `mm_to_pulses` | int | `1000` | 毫米到脉冲转换系数 |
| `max_position_mm` | int | `500` | 最大行程 (mm) |
| `min_position_mm` | int | `0` | 最小行程 (mm) |
| `default_speed_rpm` | int | `100` | 默认速度 (rpm) |
| `max_speed_rpm` | int | `200` | 最大速度限幅 (rpm) |
| `min_speed_rpm` | int | `0` | 最小速度限幅 (rpm) |
| `accel_time_ms` | int | `500` | 加速时间 (ms) |
| `decel_time_ms` | int | `500` | 减速时间 (ms) |
| `status_update_hz` | int | `10` | SDO 模式状态发布频率 (Hz) |
| `reconnect_timeout_ms` | int | `1000` | 断电重连超时 (ms) |
| `pdo_enabled` | bool | `true` | 是否启用 PDO 模式 |
| `pdo_frequency_hz` | int | `200` | PDO 控制循环频率 (Hz) |

## 话题

### 发布的话题（节点输出）

#### `lift/position` — `std_msgs/Float64`
升降当前位置，单位 mm。PDO 模式下由 TPDO 驱动 200Hz 发布；SDO 模式下按 `status_update_hz` 频率发布。

```bash
# 查看当前位置
ros2 topic echo /lift/position
```

#### `lift/speed` — `std_msgs/Float64`
升降当前速度，单位 rpm。

```bash
ros2 topic echo /lift/speed
```

#### `lift/enabled` — `std_msgs/Bool`
电机使能状态。`true` 表示电机已使能可运动，`false` 表示未使能。

```bash
ros2 topic echo /lift/enabled
```

#### `lift/target_reached` — `std_msgs/Bool`
是否已到达目标位置。`true` 表示已到位，`false` 表示运动中。

```bash
ros2 topic echo /lift/target_reached
```

#### `lift/fault` — `std_msgs/Bool`
是否处于故障状态。`true` 表示有故障，需调用 `lift/clear_error` 清除后才能继续运动。

```bash
ros2 topic echo /lift/fault
```

#### `lift/error_code` — `std_msgs/Int32`
驱动器错误码（CiA 402 对象 0x603F）。`0` 表示无错误，非零值参考驱动器手册。

```bash
ros2 topic echo /lift/error_code
```

### 订阅的话题（节点输入）

#### `lift/target_position` — `std_msgs/Float64`
设置目标位置，单位 mm。范围 `min_position_mm` ~ `max_position_mm`。

```bash
# 移动到 100mm
ros2 topic pub --once /lift/target_position std_msgs/msg/Float64 '{data: 100.0}'

# 移动到 250mm
ros2 topic pub --once /lift/target_position std_msgs/msg/Float64 '{data: 250.0}'
```

#### `lift/target_speed` — `std_msgs/Float64`
设置运动速度，单位 rpm。

```bash
# 设置速度为 80 rpm
ros2 topic pub --once /lift/target_speed std_msgs/msg/Float64 '{data: 80.0}'
```

#### `lift/target_position_with_speed` — `std_msgs/Float64MultiArray`
同时设置目标位置和运动速度。`data` 数组为 `[position_mm, speed_rpm]`，长度必须为 2。

```bash
# 移动到 150mm，速度 80 rpm
ros2 topic pub --once /lift/target_position_with_speed std_msgs/msg/Float64MultiArray '{data: [150.0, 80.0]}'

# 移动到 0mm，速度 50 rpm
ros2 topic pub --once /lift/target_position_with_speed std_msgs/msg/Float64MultiArray '{data: [0.0, 50.0]}'
```

#### `lift/enable_cmd` — `std_msgs/Bool`
使能/失能电机。`true` 使能，`false` 失能。

```bash
# 使能电机
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool '{data: true}'

# 失能电机
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool '{data: false}'
```

#### `lift/zero_cmd` — `std_msgs/Bool`
设置当前位置为零点（homing method 35，不回零）。发送 `true` 触发。

```bash
ros2 topic pub --once /lift/zero_cmd std_msgs/msg/Bool '{data: true}'
```

#### `lift/clear_error_cmd` — `std_msgs/Bool`
清除驱动器故障。发送 `true` 触发。

```bash
ros2 topic pub --once /lift/clear_error_cmd std_msgs/msg/Bool '{data: true}'
```

## 完整使用示例

```bash
# 1. 配置 CAN 接口
sudo bash ~/workspace/ros2_ws/src/lift_controller/scripts/setup_can.sh

# 2. 启动节点
ros2 launch lift_controller lift_motor.launch.py

# 3. 在另一个终端查看状态
ros2 topic echo /lift/position
ros2 topic echo /lift/enabled

# 4. 控制升降运动
ros2 topic pub --once /lift/target_position std_msgs/msg/Float64 '{data: 100.0}'

# 5. 查看所有话题
ros2 topic list
```

## 工作模式

### PDO 模式（默认，`pdo_enabled: true`）
- PP 模式（Profile Position），驱动器内部处理轨迹
- RPDO1：ControlWord + TargetPosition，200Hz 周期发送 + SYNC
- TPDO1：StatusWord + PositionActual + ErrorCode，事件驱动反馈
- 控制延迟 < 0.1ms

### SDO 模式（回退）
- PDO 配置失败时自动启用
- 状态按 `status_update_hz` 频率通过 SDO 轮询
- 控制延迟 ~5-10ms

## 节点架构

```
CanInterface (SocketCAN)
    ├── 接收线程: SDO 响应入队, TPDO 走回调
    └── sendAndWait: 队列 + condition_variable 等待 SDO 响应

MotorDriver
    ├── init: 读 F0.1.002 → 配置 PP → 配置 PDO → 使能
    ├── controlLoop (PDO): 200Hz 发送 RPDO + SYNC
    └── handleTPDO: 解析 TPDO1 → statusCallback

LiftController (ROS2 Node)
    ├── 发布: position/speed/enabled/target_reached/fault/error_code
    └── 订阅: target_position/target_speed/target_position_with_speed/enable_cmd/zero_cmd/clear_error_cmd
```
