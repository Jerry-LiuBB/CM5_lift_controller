# lift_controller

基于 ROS 2 的升降电机控制包，通过 CANopen (CiA 402) 协议控制 DS/RS 系列伺服或步进驱动器。默认使用 `can1` 连接升降电机，支持 PDO 高频控制和 SDO 回退控制。

## 功能概览

- PDO 模式下使用 PP 模式、RPDO1、TPDO1、TPDO2 和 SYNC 进行实时控制，默认控制频率为 200 Hz。
- PDO 配置失败时自动回退到 SDO 模式，状态默认按 10 Hz 发布。
- 接收线程统一处理 SDO 响应和 TPDO，SDO 请求使用队列和条件变量等待响应。
- 电机掉电或 CAN 连接异常后尝试自动重连。
- 通过驱动器 `0x60FD` 数字输入读取 LIMIT_1/LIMIT_2，支持限位停车和反方向退出。
- 支持位置、速度、使能、清错和当前位置置零控制。
- 支持位置和速度范围、加减速时间、跟随误差窗口等参数配置。

## 目录结构

```text
lift_controller/
├── config/
│   └── lift_config.yaml          # ROS 2 参数文件，默认 CAN 接口为 can1
├── include/lift_controller/
│   ├── can_interface.hpp         # SocketCAN 接口
│   ├── lift_controller.hpp       # ROS 2 节点
│   └── motor_driver.hpp           # CANopen 电机驱动
├── launch/
│   └── lift_motor.launch.py      # 启动文件
├── scripts/
│   ├── setup_can.sh              # 配置 can1 的 CAN 接口
│   └── read_params.py            # 通过 CANopen SDO 读取驱动器参数
└── src/
    ├── can_interface.cpp
    ├── lift_controller.cpp
    ├── main.cpp
    └── motor_driver.cpp
```

## 硬件和软件要求

- Linux 系统，并已加载 SocketCAN。
- 一个名称为 `can1` 的 CAN 接口。可以使用 `ip link show can1` 检查。
- DS/RS 系列驱动器，并已配置为 CANopen 模式：`F0.1.002 = 20`。
- 驱动器节点 ID 与配置文件中的 `node_id` 一致，当前默认为 `1`。
- CAN 总线两端安装 120 ohm 终端电阻。
- 升降电机、驱动器和 CAN 适配器供电正常。
- ROS 2、`rclcpp` 和 `std_msgs` 已安装或已在当前工作区编译。

## 编译

```bash
cd ~/workspace/ros2_ws
colcon build --packages-up-to lift_controller
source install/setup.bash
```

如果使用本仓库路径，请将 `~/workspace/ros2_ws` 替换为实际工作区路径。

## CAN 接口配置

升降电机默认使用 `can1`，默认波特率为 1 Mbps，采样点为 0.875。启动 ROS 2 节点前必须先配置接口。

### 使用配置脚本

```bash
# 使用默认配置: can1 @ 1000000 bps
sudo bash src/lift_controller/scripts/setup_can.sh

# 显式指定接口和波特率
sudo bash src/lift_controller/scripts/setup_can.sh can1 1000000
```

脚本会执行以下操作：

1. 将 `can1` 置为 down。
2. 设置 CAN 波特率和采样点。
3. 将 `can1` 置为 up。
4. 设置 bus-off 自动恢复，恢复时间为 100 ms。
5. 输出 CAN 状态和错误计数。

### 手动配置

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 sample-point 0.875
sudo ip link set can1 type can restart-ms 100
sudo ip link set can1 up
ip -details link show can1
```

### CAN 检查

```bash
# 检查接口是否存在及是否已启动
ip link show can1

# 监听 CAN 总线，确认驱动器有报文
candump can1

# 查看驱动器参数，脚本内默认使用 can1 和节点 ID 1
python3 src/lift_controller/scripts/read_params.py
```

如果 `can1` 不存在，请先确认 CAN 适配器驱动和接口命名。若显示 `BUS-OFF`、`ERROR-PASSIVE` 或没有任何响应，应检查驱动器供电、CANH/CANL 接线、终端电阻、节点 ID 和波特率。

## 启动

```bash
source install/setup.bash
ros2 launch lift_controller lift_motor.launch.py
```

启动文件会优先读取源码目录中的 `src/lift_controller/config/lift_config.yaml`，因此修改源码配置后重新启动即可生效。若源码配置不存在，则回退到安装目录中的配置文件。

也可以在启动时覆盖参数：

```bash
ros2 launch lift_controller lift_motor.launch.py \
  can_interface:=can1 \
  can_bitrate:=1000000 \
  node_id:=1
```

节点启动后会在日志中打印类似以下内容：

```text
[CAN] Connected to can1
[lift_controller] Lift Controller started (PDO mode)
```

## 配置参数

配置文件位于 [config/lift_config.yaml](config/lift_config.yaml)。以下是当前配置文件中的参数：

| 参数 | 类型 | 当前值 | 说明 |
|------|------|--------|------|
| `can_interface` | string | `"can1"` | SocketCAN 接口名，升降电机使用 `can1` |
| `can_bitrate` | int | `1000000` | CAN 波特率，实际接口波特率由 `ip link` 或 `setup_can.sh` 设置 |
| `node_id` | int | `1` | 驱动器 CANopen 节点 ID |
| `mm_to_pulses` | int | `1000` | 位置换算系数，脉冲数 = 毫米数 x 此参数 |
| `max_position_mm` | int | `500` | 允许的最大目标位置，单位 mm |
| `min_position_mm` | int | `0` | 允许的最小目标位置，单位 mm |
| `default_speed_rpm` | int | `200` | 驱动器默认速度，单位 rpm |
| `max_speed_rpm` | int | `1000` | 允许设置的最大速度，单位 rpm |
| `min_speed_rpm` | int | `0` | 允许设置的最小速度，单位 rpm |
| `accel_time_ms` | int | `1000` | 加速时间，单位 ms |
| `decel_time_ms` | int | `1000` | 减速时间，单位 ms |
| `following_error_pulses` | int | `200000` | 跟随误差窗口，单位脉冲；设为 `0` 表示使用驱动器内部值 |
| `max_motor_speed_rpm` | int | `0` | 电机最大转速，设为 `0` 表示使用驱动器内部值 |
| `status_update_hz` | int | `10` | SDO 模式下状态发布频率，单位 Hz |
| `reconnect_timeout_ms` | int | `1000` | 掉电重连超时时间，单位 ms |
| `pdo_enabled` | bool | `true` | 是否启用 PDO 模式；失败后仍会自动尝试 SDO 回退 |
| `pdo_frequency_hz` | int | `200` | PDO 控制循环频率，单位 Hz |
| `limit_safety_enabled` | bool | `true` | 是否启用驱动器数字输入限位安全联锁 |
| `limit_1_direction` | string | `"min"` | LIMIT_1 禁止的运动方向，可选 `min`、`max`、`none` |
| `limit_2_direction` | string | `"max"` | LIMIT_2 禁止的运动方向，可选 `min`、`max`、`none` |
| `limit_1_input_mask` | int | `1` | 驱动器 `0x60FD` 中 LIMIT_1 对应 bit mask，需现场实测确认 |
| `limit_2_input_mask` | int | `2` | 驱动器 `0x60FD` 中 LIMIT_2 对应 bit mask，需现场实测确认 |
| `limit_inputs_active_high` | bool | `true` | `true` 表示 mask 位为 1 时触发，`false` 表示 mask 位为 0 时触发 |
| `limit_status_timeout_sec` | double | `0.5` | 收到过驱动器数字输入后，超过此时间没有新状态则安全停车；小于等于 `0` 表示关闭超时检测 |

`limit_1_direction` 和 `limit_2_direction` 只决定对应限位禁止的目标方向。触发限位后会立即停车，但允许发送反方向目标位置用于退出限位。驱动器数字输入状态不可用、mask 配置无效或状态超时会进入安全故障，位置、带速度的位置、使能和置零等相关命令会被拒绝。

## ROS 2 话题

以下示例假设节点使用默认名称 `lift_controller`，话题均使用绝对名称。ROS 2 命令中的消息类型必须使用完整写法，例如 `std_msgs/msg/Float64`。

### 发布话题

#### `/lift/position` - `std_msgs/msg/Float64`

当前位置，单位 mm。PDO 模式下通常由 TPDO 反馈驱动发布，SDO 模式下按 `status_update_hz` 发布。

```bash
ros2 topic echo /lift/position
ros2 topic info /lift/position
```

示例输出：

```text
data: 123.5
```

#### `/lift/speed` - `std_msgs/msg/Float64`

电机实际反馈速度，单位 rpm。正负号表示运动方向，具体方向以现场机械安装和驱动器定义为准。

```bash
ros2 topic echo /lift/speed
```

示例输出：

```text
data: 80.0
```

#### `/lift/speed_rpm` - `std_msgs/msg/Float64`

当前设定的运动速度，单位 rpm。该值由 `lift/target_speed` 或 `lift/target_position_with_speed` 更新，不等同于实际反馈速度。

```bash
ros2 topic echo /lift/speed_rpm
```

示例输出：

```text
data: 200.0
```

#### `/lift/speed_percent` - `std_msgs/msg/Float64`

当前设定速度相对于 `max_speed_rpm` 的百分比，范围通常为 0 到 100。

```bash
ros2 topic echo /lift/speed_percent
```

示例输出：

```text
data: 20.0
```

#### `/lift/enabled` - `std_msgs/msg/Bool`

电机使能状态。`true` 表示驱动器已使能，`false` 表示未使能。

```bash
ros2 topic echo /lift/enabled
```

示例输出：

```text
data: true
```

#### `/lift/target_reached` - `std_msgs/msg/Bool`

目标位置到达状态。`true` 表示驱动器报告已到位，`false` 表示仍在运动或尚未到达目标。

```bash
ros2 topic echo /lift/target_reached
```

示例输出：

```text
data: false
```

#### `/lift/fault` - `std_msgs/msg/Bool`

驱动器故障状态。`true` 表示当前存在故障，通常需要先发送 `lift/clear_error_cmd` 清错，再根据驱动器状态重新使能。

```bash
ros2 topic echo /lift/fault
```

示例输出：

```text
data: false
```

#### `/lift/error_code` - `std_msgs/msg/Int32`

驱动器错误码，对应 CiA 402 对象 `0x603F`。`0` 表示无错误，非零值需要结合驱动器手册解释。

```bash
ros2 topic echo /lift/error_code
```

示例输出：

```text
data: 0
```

#### `/lift/following_error` - `std_msgs/msg/Bool`

跟随误差状态。`true` 表示实际位置与目标位置的误差超过驱动器或配置的跟随误差窗口。

```bash
ros2 topic echo /lift/following_error
```

示例输出：

```text
data: false
```

#### `/lift/limit_stop` - `std_msgs/msg/Bool`

限位安全停车状态。`true` 表示 LIMIT_1、LIMIT_2、驱动器数字输入不可用、mask 配置无效或状态超时触发了安全停车；`false` 表示当前没有活动的安全停车状态。该话题只在状态发生变化时发布。

```bash
ros2 topic echo /lift/limit_stop
```

示例输出：

```text
data: true
```

#### `/lift/version_info` - `std_msgs/msg/String`

驱动器版本信息。该话题使用 transient local QoS，节点启动后才加入的订阅者也可以收到最近一次版本信息。

```bash
ros2 topic echo /lift/version_info
ros2 topic echo /lift/version_info --qos-durability transient_local
```

示例输出：

```text
data: "DS/RS ..."
```

### 订阅话题

#### `/lift/target_position` - `std_msgs/msg/Float64`

设置目标位置，单位 mm。代码会检查 `min_position_mm` 到 `max_position_mm` 的范围，并根据限位安全状态拒绝不允许的运动方向。

```bash
# 移动到 100 mm
ros2 topic pub --once /lift/target_position std_msgs/msg/Float64 "{data: 100.0}"

# 移动到最大位置 500 mm
ros2 topic pub --once /lift/target_position std_msgs/msg/Float64 "{data: 500.0}"
```

#### `/lift/target_speed` - `std_msgs/msg/Float64`

设置运动速度，单位 rpm。超出 `min_speed_rpm` 到 `max_speed_rpm` 时节点会记录警告，并由电机驱动器执行限幅。

```bash
# 设置速度为 80 rpm
ros2 topic pub --once /lift/target_speed std_msgs/msg/Float64 "{data: 80.0}"
```

#### `/lift/target_position_with_speed` - `std_msgs/msg/Float64MultiArray`

同时设置目标位置和速度。`data` 必须至少包含两个元素，格式为 `[position_mm, speed_rpm]`。

```bash
# 移动到 150 mm，速度 80 rpm
ros2 topic pub --once /lift/target_position_with_speed \
  std_msgs/msg/Float64MultiArray "{data: [150.0, 80.0]}"

# 移动到 0 mm，速度 50 rpm
ros2 topic pub --once /lift/target_position_with_speed \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 50.0]}"
```

#### `/lift/enable_cmd` - `std_msgs/msg/Bool`

使能或失能电机。发送 `true` 使能，发送 `false` 失能。驱动器数字输入安全故障有效时，`true` 命令会被拒绝。

```bash
# 使能电机
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool "{data: true}"

# 失能电机
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool "{data: false}"
```

#### `/lift/zero_cmd` - `std_msgs/msg/Bool`

当前位置置零。发送 `true` 会调用驱动器的 homing method 35，不执行物理回零运动。驱动器数字输入安全故障有效时命令会被拒绝。

```bash
ros2 topic pub --once /lift/zero_cmd std_msgs/msg/Bool "{data: true}"
```

#### `/lift/clear_error_cmd` - `std_msgs/msg/Bool`

清除驱动器故障。发送 `true` 触发一次清错操作，发送 `false` 不执行动作。

```bash
ros2 topic pub --once /lift/clear_error_cmd std_msgs/msg/Bool "{data: true}"
```

#### `/lift/drive_digital_inputs` - `std_msgs/msg/Int32`

驱动器数字输入状态，来自 CANopen 对象字典 `0x60FD Digital input`。PDO 模式下由 TPDO2 上报，SDO 回退模式下由状态轮询读取。

查看驱动器数字输入：

```bash
ros2 topic echo /lift/drive_digital_inputs
```

现场调试 LIMIT_1/LIMIT_2 的 bit mask 时，建议分别记录三种状态：

```bash
ros2 topic echo /lift/drive_digital_inputs --once
# 不触发限位、只触发 LIMIT_1、只触发 LIMIT_2 时各读一次
```

用三次读数的变化位确定 `limit_1_input_mask` 和 `limit_2_input_mask`。例如正常为 `0`，触发 LIMIT_1 后为 `1`，触发 LIMIT_2 后为 `2`，则保持默认：

```yaml
limit_1_input_mask: 1
limit_2_input_mask: 2
```

## 完整运行示例

建议按以下顺序在不同终端执行：

```bash
# 终端 1: 编译并加载工作区
cd ~/workspace/ros2_ws
colcon build --packages-up-to lift_controller
source install/setup.bash

# 终端 2: 配置升降电机 CAN，默认是 can1 @ 1 Mbps
sudo bash src/lift_controller/scripts/setup_can.sh

# 终端 3: 启动升降节点
source install/setup.bash
ros2 launch lift_controller lift_motor.launch.py

# 终端 4: 观察状态
source install/setup.bash
ros2 topic echo /lift/position
ros2 topic echo /lift/enabled
ros2 topic echo /lift/fault
ros2 topic echo /lift/limit_stop

# 终端 5: 发送控制命令
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /lift/target_position_with_speed \
  std_msgs/msg/Float64MultiArray "{data: [100.0, 80.0]}"
```

停止运动或停止发送目标位置后，可以发送失能命令：

```bash
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool "{data: false}"
```

## 工作模式

### PDO 模式

当 `pdo_enabled: true` 且驱动器支持所需 PDO 配置时，节点会进入 PDO 模式：

- 使用 PP 模式进行位置控制。
- RPDO1 发送 ControlWord 和 TargetPosition。
- TPDO1 返回 StatusWord、PositionActual 和 ErrorCode。
- TPDO2 返回 VelocityActual。
- 默认控制循环频率为 200 Hz。

### SDO 回退模式

当 PDO 配置失败时，节点会自动回退到 SDO 模式：

- 通过 SDO 读写控制字、目标位置和状态。
- 状态按 `status_update_hz` 发布，默认 10 Hz。
- 控制延迟高于 PDO 模式，适合调试和兼容性验证。

## 常见问题

### 节点提示找不到 `can1`

```bash
ip link show can1
```

如果没有输出，说明系统没有创建该接口。请检查 CAN 适配器驱动、设备连接和接口命名。

### CAN 状态为 `BUS-OFF` 或没有驱动器响应

依次检查：

1. 驱动器和 CAN 适配器是否供电。
2. CANH、CANL 是否正确连接。
3. 总线两端是否各有 120 ohm 终端电阻。
4. `can1` 波特率是否为 1 Mbps。
5. `node_id` 是否与驱动器节点 ID 一致。
6. 驱动器是否设置为 CANopen 模式。

可以使用以下命令观察总线：

```bash
candump can1
ip -details link show can1
```

### 触发限位后不能继续运动

这是限位安全联锁的预期行为。确认 `/lift/limit_stop` 和 `/lift/drive_digital_inputs`：

```bash
ros2 topic echo /lift/limit_stop
ros2 topic echo /lift/drive_digital_inputs
```

触发限位时，继续朝限位方向的目标位置会被拒绝；发送反方向目标位置可以退出限位。如果驱动器数字输入不可用、mask 配置无效或状态超时，需要先恢复 CAN 通信和驱动器输入状态。

### 驱动器有故障

先查看故障状态和错误码：

```bash
ros2 topic echo /lift/fault
ros2 topic echo /lift/error_code
```

确认机械和电气条件正常后清除故障：

```bash
ros2 topic pub --once /lift/clear_error_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /lift/enable_cmd std_msgs/msg/Bool "{data: true}"
```

## 节点架构

```text
CanInterface (SocketCAN on can1)
    ├── 接收线程: SDO 响应入队，TPDO 通过回调分发
    └── sendAndWait: 队列和 condition_variable 等待 SDO 响应

MotorDriver
    ├── init: 读取驱动器模式，配置 PP 和 PDO，执行使能
    ├── controlLoop (PDO): 发送 RPDO 和 SYNC
    └── handleTPDO: 解析状态、位置、错误码、速度和数字输入

LiftController (ROS 2 Node)
    ├── 发布: position/speed/speed_rpm/speed_percent/enabled
    │          target_reached/fault/error_code/following_error
    │          limit_stop/version_info/drive_digital_inputs
    └── 订阅: target_position/target_speed/target_position_with_speed
               enable_cmd/zero_cmd/clear_error_cmd
```
