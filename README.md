# lift_controller

基于 ROS2 的升降电机控制包，通过 CANopen (CiA 402) 协议控制 DS/RS 系列伺服/步进驱动器，支持 200Hz 高频 PDO 实时控制。

## 功能特性

- **PDO 高频控制**：CSP/PP 模式 + RPDO1 + TPDO1 + SYNC，控制频率可达 200Hz
- **SDO 回退模式**：PDO 配置失败时自动回退到 SDO 模式（~10Hz），保证可用性
- **队列式 CAN 接收**：SDO 响应入队 + condition_variable 等待，避免竞态条件
- **断电重连**：电机掉电后自动检测并重连
- **限位安全联锁**：订阅电源板限位状态，LIMIT_1/LIMIT_2 触发时立即停止升降运动
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
colcon build --packages-up-to lift_controller
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
| `limit_safety_enabled` | bool | `true` | 是否启用电源板限位安全联锁 |
| `power_board_status_topic` | string | `"/power_board/status"` | 电源板状态话题 |
| `limit_1_direction` | string | `"min"` | LIMIT_1 对应的限位方向；`min` 表示禁止继续向 `min_position_mm` 方向运动，`max` 表示禁止继续向 `max_position_mm` 方向运动，`none` 表示不参与限位保护 |
| `limit_2_direction` | string | `"max"` | LIMIT_2 对应的限位方向；现场接线相反时可与 `limit_1_direction` 对调 |
| `limit_stop_on_power_board_fault` | bool | `true` | 电源板通信异常或采集停滞时是否停车 |
| `limit_status_timeout_sec` | double | `0.5` | 已收到过电源板状态后，状态超时多久触发安全停车；小于等于 0 表示禁用超时检测 |

## 话题

### 发布的话题（节点输出）

#### `lift/position` — `std_msgs/Float64`
升降当前位置，单位 mm。PDO 模式下由 TPDO 驱动 200Hz 发布；SDO 模式下按 `status_update_hz` 频率发布。

```bash
# 查看当前位置
ros2 topic echo /lift/position
```

#### `lift/speed` — `std_msgs/Float64`
升降当前实际速度（电机反馈），单位 rpm。PDO 模式下由 TPDO2 驱动 200Hz 发布。

```bash
ros2 topic echo /lift/speed
```

#### `lift/speed_rpm` — `std_msgs/Float64`
当前设定的运动速度（PP 模式 profile velocity），单位 rpm。通过 `lift/target_speed` 或 `lift/target_position_with_speed` 设置。

```bash
ros2 topic echo /lift/speed_rpm
```

#### `lift/speed_percent` — `std_msgs/Float64`
当前设定速度占最大速度（`max_speed_rpm`）的百分比，范围 0~100。

```bash
ros2 topic echo /lift/speed_percent
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

#### `lift/limit_stop` — `std_msgs/Bool`
限位安全停车状态。`true` 表示 LIMIT_1、LIMIT_2、电源板通信异常、采集停滞或状态超时触发了安全停车。限位触发后会立即请求停车；之后只拒绝继续朝触发限位方向的目标位置命令，允许反方向目标位置用于退出限位。

```bash
ros2 topic echo /lift/limit_stop
```

### 订阅的话题（节点输入）

#### `/power_board/status` — `power_board_msgs/PowerBoardStatus`
电源板状态输入。升降节点读取其中的 `limit_1_triggered` 和 `limit_2_triggered`，任一限位触发时立即请求电机停车，并按 `limit_1_direction` / `limit_2_direction` 判断后续目标位置是否允许反向退出。

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
- TPDO2：VelocityActual，同步反馈（TPDO1 已满 8 字节，速度单独走 TPDO2）
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
    └── handleTPDO: 解析 TPDO1(StatusWord/Position/ErrorCode) + TPDO2(Velocity) → statusCallback

LiftController (ROS2 Node)
    ├── 发布: position/speed/enabled/target_reached/fault/error_code
    └── 订阅: target_position/target_speed/target_position_with_speed/enable_cmd/zero_cmd/clear_error_cmd
```
