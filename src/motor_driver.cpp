#include "lift_controller/motor_driver.hpp"
#include <iostream>
#include <cstring>
#include <cmath>
#include <chrono>

namespace lift_controller {

// CiA 402 对象字典
constexpr uint16_t OD_CONTROLWORD     = 0x6040;
constexpr uint16_t OD_STATUSWORD      = 0x6041;
constexpr uint16_t OD_TARGET_VELOCITY  = 0x60FF;
constexpr uint16_t OD_MODES_OP        = 0x6060;
constexpr uint16_t OD_PROFILE_ACC     = 0x6083;
constexpr uint16_t OD_PROFILE_DEC      = 0x6084;
constexpr uint16_t OD_ERROR_CODE      = 0x603F;
constexpr uint16_t OD_TARGET_POSITION = 0x607A;
constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;
constexpr uint16_t OD_POSITION_ACTUAL = 0x6063;
constexpr uint16_t OD_VELOCITY_ACTUAL = 0x606C;
constexpr uint16_t OD_DIGITAL_INPUTS  = 0x60FD;
constexpr uint16_t OD_HOME_OFFSET     = 0x607C;
constexpr uint16_t OD_HOMING_METHOD   = 0x6098;
constexpr uint16_t OD_POSITION_LIMIT  = 0x607D;  // 软件位置限位
constexpr uint16_t OD_FOLLOWING_ERR   = 0x6065;  // 跟随误差窗口
constexpr uint16_t OD_MAX_MOTOR_SPEED = 0x6080;  // 电机最大转速
constexpr uint16_t OD_D18_VERSION     = 0x5018;  // D18: 驱动器版本信息(A轴)
constexpr uint16_t OD_D19_VERSION     = 0x5019;  // D19: 驱动器版本信息(A轴)
constexpr uint16_t OD_POSITION_WINDOW = 0x6068;  // 位置窗口(到达判定的脉冲范围)
constexpr uint16_t OD_F020E_POS_DEV   = 0x220E;  // F0.2.014: 厂商位置到达偏差

// PDO 通信/映射参数
constexpr uint16_t OD_RPDO1_COMM     = 0x1400;
constexpr uint16_t OD_RPDO1_MAP      = 0x1600;
constexpr uint16_t OD_TPDO1_COMM     = 0x1800;
constexpr uint16_t OD_TPDO1_MAP      = 0x1A00;
constexpr uint16_t OD_TPDO2_COMM     = 0x1801;
constexpr uint16_t OD_TPDO2_MAP      = 0x1A01;

// 驱动器厂商参数
constexpr uint16_t OD_CONTROL_MODE    = 0x2102;  // F0.1.002: 20=CANopen模式

MotorDriver::MotorDriver(CanInterface& can_iface)
    : can_iface_(can_iface), initialized_(false), running_(false), connected_(false),
      pdo_mode_(false), control_running_(false), reconnect_running_(false) {
    std::memset(&status_, 0, sizeof(status_));
    pdo_control_word_ = 0;
    pdo_target_position_ = 0;
    pdo_trigger_pending_ = false;
    current_speed_rpm_ = 0;
}

MotorDriver::~MotorDriver() { shutdown(); }

void MotorDriver::setConfig(const MotorConfig& config) {
    config_ = config;
}

bool MotorDriver::init(const MotorConfig& config) {
    if (initialized_) return true;
    config_ = config;
    connected_ = can_iface_.is_connected();

    if (!connected_) {
        std::cerr << "[Motor] CAN not connected" << std::endl;
        return false;
    }

    // 检查驱动器是否处于 CANopen 模式 (F0.1.002 = 20)
    uint32_t mode_val;
    if (readSDO(OD_CONTROL_MODE, 0, mode_val)) {
        if (mode_val != 20) {
            std::cerr << "[Motor] WARNING: Drive not in CANopen mode (F0.1.002=" << mode_val
                      << ", need 20). Set F0.1.002=20 and power cycle." << std::endl;
        }
    } else {
        std::cerr << "[Motor] Cannot read F0.1.002 — drive may not be in CANopen mode."
                  << "\n  Check: 1) CAN wiring  2) Drive power  3) F0.1.002=20  4) Node ID="
                  << config_.node_id << std::endl;
    }

    // SDO 配置驱动参数 (PP 模式)
    if (!reconfigureDrive()) {
        std::cerr << "[Motor] Failed to configure drive" << std::endl;
        return false;
    }

    // 读取驱动器版本信息
    readVersionInfo();

    // 读取位置窗口参数(诊断用)
    uint32_t pos_window = 0, pos_dev = 0;
    if (readSDO(OD_POSITION_WINDOW, 0, pos_window)) {
        std::cout << "[Motor] Position window (0x6068): " << static_cast<int32_t>(pos_window)
                  << " pulses (" << static_cast<int32_t>(pos_window) / config_.mm_to_pulses << "mm)" << std::endl;
    }
    if (readSDO(OD_F020E_POS_DEV, 0, pos_dev)) {
        std::cout << "[Motor] Position deviation (0x220E/F0.2.014): " << static_cast<int32_t>(pos_dev) << std::endl;
    }

    // 尝试 PDO 模式
    if (config_.pdo_enabled) {
        if (configurePDO()) {
            pdo_mode_ = true;
            std::cout << "[Motor] PDO mode configured (PP, "
                      << config_.pdo_frequency_hz << " Hz)" << std::endl;
        } else {
            std::cerr << "[Motor] PDO config failed, falling back to SDO mode" << std::endl;
            pdo_mode_ = false;
        }
    } else {
        pdo_mode_ = false;
    }

    initialized_ = true;
    running_ = true;

    // 注册 TPDO 接收回调
    can_iface_.setReceiveCallback(
        [this](const CanFrame& frame) { this->handleTPDO(frame); });

    if (pdo_mode_) {
        // 读取当前位置作为初始目标
        uint32_t pos;
        if (readSDO(OD_POSITION_ACTUAL, 0, pos)) {
            pdo_target_position_ = static_cast<int32_t>(pos);
        }
        pdo_control_word_ = 0x0F;
        pdo_trigger_pending_ = false;

        // 通过 SDO 使能驱动
        enable();

        {
            std::lock_guard<std::mutex> lock(tpdo_time_mutex_);
            last_tpdo_time_ = std::chrono::steady_clock::now();
        }
        startPDOMode();

        // PDO 模式下 statusUpdateLoop 不运行,启动独立重连看门狗检测断电
        reconnect_running_ = true;
        reconnect_thread_ = std::thread(&MotorDriver::reconnectLoop, this);
    } else {
        status_thread_ = std::thread(&MotorDriver::statusUpdateLoop, this);
    }

    std::cout << "[Motor] Initialized, Node: " << config_.node_id
              << (pdo_mode_ ? " (PDO)" : " (SDO)") << std::endl;
    return true;
}

void MotorDriver::readVersionInfo() {
    // 读取 D18(0x5018) 和 D19(0x5019) — 驱动器版本信息
    uint32_t d18 = 0, d19 = 0;
    bool ok18 = readSDO(OD_D18_VERSION, 0, d18);
    bool ok19 = readSDO(OD_D19_VERSION, 0, d19);

    if (ok18 || ok19) {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "D18=0x%08X(%u), D19=0x%08X(%u)",
                 d18, d18, d19, d19);
        version_info_ = buf;
        std::cout << "[Motor] Version: " << version_info_ << std::endl;
        if (!ok18) std::cerr << "[Motor] WARNING: Failed to read D18 (0x5018)" << std::endl;
        if (!ok19) std::cerr << "[Motor] WARNING: Failed to read D19 (0x5019)" << std::endl;
    } else {
        version_info_ = "Unknown (SDO read failed)";
        std::cerr << "[Motor] WARNING: Failed to read version info (D18/D19)" << std::endl;
    }
}

bool MotorDriver::reconfigureDrive() {
    // PP 模式 (mode=1) — 驱动器内部处理轨迹
    if (!writeSDO(OD_MODES_OP, 0, 1, 1)) {
        std::cerr << "[Motor] Failed to set PP mode (1)" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!writeSDO(OD_PROFILE_ACC, 0, static_cast<uint32_t>(config_.accel_time_ms), 4)) {
        std::cerr << "[Motor] Failed to set acceleration" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!writeSDO(OD_PROFILE_DEC, 0, static_cast<uint32_t>(config_.decel_time_ms), 4)) {
        std::cerr << "[Motor] Failed to set deceleration" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (config_.default_speed_rpm > 0) {
        writeSDO(OD_PROFILE_VELOCITY, 0,
                 static_cast<uint32_t>(config_.default_speed_rpm * 10), 4);
        current_speed_rpm_ = config_.default_speed_rpm;
    }

    // 设置驱动器软件位置限位(与 YAML 配置一致)
    int32_t min_pulses = config_.min_position_mm * config_.mm_to_pulses;
    int32_t max_pulses = config_.max_position_mm * config_.mm_to_pulses;
    if (!writeSDO(OD_POSITION_LIMIT, 1, static_cast<uint32_t>(min_pulses), 4)) {
        std::cerr << "[Motor] WARNING: Failed to set min position limit (0x607D:01)" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_POSITION_LIMIT, 2, static_cast<uint32_t>(max_pulses), 4)) {
        std::cerr << "[Motor] WARNING: Failed to set max position limit (0x607D:02)" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 设置跟随误差窗口(0x6065) — 高速运动时需要足够大的窗口
    if (config_.following_error_pulses > 0) {
        if (writeSDO(OD_FOLLOWING_ERR, 0,
                     static_cast<uint32_t>(config_.following_error_pulses), 4)) {
            // 读回验证
            uint32_t few_read;
            if (readSDO(OD_FOLLOWING_ERR, 0, few_read)) {
                std::cout << "[Motor] Following error window: set=" << config_.following_error_pulses
                          << ", readback=" << static_cast<int32_t>(few_read)
                          << " (" << static_cast<int32_t>(few_read) / config_.mm_to_pulses << "mm)" << std::endl;
                if (static_cast<int32_t>(few_read) != config_.following_error_pulses) {
                    std::cerr << "[Motor] WARNING: Following error window readback mismatch! "
                              << "Drive may cap the value." << std::endl;
                }
            } else {
                std::cout << "[Motor] Following error window: " << config_.following_error_pulses
                          << " pulses (readback failed)" << std::endl;
            }
        } else {
            std::cerr << "[Motor] WARNING: Failed to set following error window (0x6065)" << std::endl;
        }
    } else {
        uint32_t few;
        if (readSDO(OD_FOLLOWING_ERR, 0, few)) {
            std::cout << "[Motor] Current following error window: " << static_cast<int32_t>(few)
                      << " pulses (" << static_cast<int32_t>(few) / config_.mm_to_pulses << "mm)" << std::endl;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 设置电机最大转速(0x6080)
    if (config_.max_motor_speed_rpm > 0) {
        writeSDO(OD_MAX_MOTOR_SPEED, 0,
                 static_cast<uint32_t>(config_.max_motor_speed_rpm), 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // 读回最大电机转速
    uint32_t mms;
    if (readSDO(OD_MAX_MOTOR_SPEED, 0, mms)) {
        std::cout << "[Motor] Max motor speed: " << mms << " rpm" << std::endl;
    }

    // 读回验证位置限位
    uint32_t read_min, read_max;
    if (readSDO(OD_POSITION_LIMIT, 1, read_min) && readSDO(OD_POSITION_LIMIT, 2, read_max)) {
        std::cout << "[Motor] Position limits: min=" << static_cast<int32_t>(read_min) / config_.mm_to_pulses
                  << "mm, max=" << static_cast<int32_t>(read_max) / config_.mm_to_pulses << "mm" << std::endl;
    } else {
        std::cerr << "[Motor] WARNING: Cannot read back position limits — drive may not support 0x607D" << std::endl;
    }

    // 设置位置窗口(0x6068) — 实际位置在此范围内时 target reached 置位
    if (writeSDO(OD_POSITION_WINDOW, 0, 100, 4)) {
        std::cout << "[Motor] Position window (0x6068): set to 100 pulses" << std::endl;
    } else {
        std::cerr << "[Motor] WARNING: Failed to set position window (0x6068)" << std::endl;
    }

    return true;
}

bool MotorDriver::configurePDO() {
    // 进入 Pre-Operational 配置 PDO
    if (!sendNMT(0x80)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // === RPDO1: ControlWord(16) + TargetPosition(32) ===
    if (!writeSDO(OD_RPDO1_COMM, 2, 1, 1)) return false;  // 传输类型=同步
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!writeSDO(OD_RPDO1_MAP, 0, 0, 1)) return false;  // 禁用映射
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_RPDO1_MAP, 1, 0x60400010, 4)) return false;  // ControlWord 16bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_RPDO1_MAP, 2, 0x607A0020, 4)) return false;  // TargetPosition 32bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_RPDO1_MAP, 0, 2, 1)) return false;  // 启用(2个对象)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // === TPDO1: StatusWord(16) + PositionActual(32) + ErrorCode(16) ===
    if (!writeSDO(OD_TPDO1_COMM, 2, 1, 1)) return false;  // 传输类型=同步
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!writeSDO(OD_TPDO1_MAP, 0, 0, 1)) return false;  // 禁用映射
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO1_MAP, 1, 0x60410010, 4)) return false;  // StatusWord 16bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO1_MAP, 2, 0x60630020, 4)) return false;  // PositionActual 32bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO1_MAP, 3, 0x603F0010, 4)) return false;  // ErrorCode 16bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO1_MAP, 0, 3, 1)) return false;  // 启用(3个对象)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // === TPDO2: VelocityActual(32) + DigitalInputs(32) ===
    // TPDO1 已满 8 字节,速度实际值和驱动器数字输入单独走 TPDO2
    if (!writeSDO(OD_TPDO2_COMM, 2, 1, 1)) return false;  // 传输类型=同步
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!writeSDO(OD_TPDO2_MAP, 0, 0, 1)) return false;  // 禁用映射
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO2_MAP, 1, 0x606C0020, 4)) return false;  // VelocityActual 32bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO2_MAP, 2, 0x60FD0020, 4)) return false;  // DigitalInputs 32bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_TPDO2_MAP, 0, 2, 1)) return false;  // 启用(2个对象)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 进入 Operational
    if (!sendNMT(0x01)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

void MotorDriver::shutdown() {
    running_ = false;
    reconnect_running_ = false;
    if (reconnect_thread_.joinable()) reconnect_thread_.join();
    stopPDOMode();
    if (status_thread_.joinable()) status_thread_.join();
    if (connected_) disable();
    can_iface_.setReceiveCallback(nullptr);
    initialized_ = false;
}

bool MotorDriver::writeSDO(uint16_t index, uint8_t subindex, uint32_t data, uint8_t length) {
    if (!connected_) return false;

    CanFrame frame;
    frame.id = 0x600 + config_.node_id;
    frame.dlc = 8;

    uint8_t cmd;
    switch (length) {
        case 1: cmd = 0x2F; break;
        case 2: cmd = 0x2B; break;
        case 4: cmd = 0x23; break;
        default: return false;
    }

    frame.data[0] = cmd;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    std::memcpy(&frame.data[4], &data, length);
    std::memset(&frame.data[4 + length], 0, 4 - length);

    CanFrame rx_frame;
    if (!can_iface_.sendAndWait(frame, rx_frame, 0x580 + config_.node_id, 200)) {
        return false;
    }
    if ((rx_frame.data[0] & 0xFF) == 0x80) {
        uint32_t abort_code = 0;
        std::memcpy(&abort_code, &rx_frame.data[4], 4);
        std::cerr << "[Motor] SDO write aborted: index=0x" << std::hex << index
                  << " code=0x" << abort_code << std::dec << std::endl;
        return false;
    }
    return true;
}

bool MotorDriver::readSDO(uint16_t index, uint8_t subindex, uint32_t& data) {
    if (!connected_) return false;

    CanFrame frame;
    frame.id = 0x600 + config_.node_id;
    frame.dlc = 8;
    frame.data[0] = 0x40;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    std::memset(&frame.data[4], 0, 4);

    CanFrame rx_frame;
    if (!can_iface_.sendAndWait(frame, rx_frame, 0x580 + config_.node_id, 200)) {
        return false;
    }

    uint8_t cmd = rx_frame.data[0];
    if (cmd == 0x80) {
        uint32_t abort_code = 0;
        std::memcpy(&abort_code, &rx_frame.data[4], 4);
        std::cerr << "[Motor] SDO read aborted: index=0x" << std::hex << index
                  << " code=0x" << abort_code << std::dec << std::endl;
        return false;
    }

    if (cmd & 0x02) {
        uint8_t n = (cmd >> 2) & 0x03;
        size_t valid_bytes = 4 - n;
        data = 0;
        std::memcpy(&data, &rx_frame.data[4], valid_bytes);
        return true;
    }
    return false;
}

bool MotorDriver::readSDO16(uint16_t index, uint8_t subindex, uint16_t& data) {
    uint32_t temp;
    if (!readSDO(index, subindex, temp)) return false;
    data = static_cast<uint16_t>(temp);
    return true;
}

bool MotorDriver::sendControlWord(uint16_t value) {
    return writeSDO(OD_CONTROLWORD, 0, value, 2);
}

bool MotorDriver::sendNMT(uint8_t command) {
    CanFrame frame;
    frame.id = 0x000;
    frame.dlc = 2;
    frame.data[0] = command;
    frame.data[1] = static_cast<uint8_t>(config_.node_id);
    return can_iface_.send(frame);
}

void MotorDriver::startPDOMode() {
    if (control_running_) return;
    control_running_ = true;
    control_thread_ = std::thread(&MotorDriver::controlLoop, this);
}

void MotorDriver::stopPDOMode() {
    control_running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
}

void MotorDriver::controlLoop() {
    // 200Hz 控制循环:每周期发 RPDO(ControlWord + TargetPosition) + SYNC
    auto period = std::chrono::microseconds(1000000 / config_.pdo_frequency_hz);
    auto next_deadline = std::chrono::steady_clock::now() + period;

    while (control_running_) {
        // PP 模式:bit4 保持 set 直到驱动器回 bit12(setpoint acknowledge)
        uint16_t cw = pdo_control_word_.load(std::memory_order_relaxed);
        if (pdo_trigger_pending_.exchange(false)) {
            pdo_bit4_remaining_.store(50);  // 最多保持50个周期(250ms@200Hz)
        }
        if (pdo_bit4_remaining_.load() > 0) {
            cw |= 0x10;  // 保持 bit4 set
            pdo_bit4_remaining_.fetch_sub(1);
        }

        // 构建 RPDO: ControlWord(2) + TargetPosition(4) = 6 bytes
        CanFrame rpdo;
        rpdo.id = 0x200 + config_.node_id;
        rpdo.dlc = 6;
        int32_t tp = pdo_target_position_.load(std::memory_order_relaxed);
        std::memcpy(&rpdo.data[0], &cw, 2);
        std::memcpy(&rpdo.data[2], &tp, 4);
        can_iface_.send(rpdo);

        // SYNC (COB-ID=0x80)
        CanFrame sync;
        sync.id = 0x80;
        sync.dlc = 0;
        can_iface_.send(sync);

        // TPDO 超时检测
        {
            std::lock_guard<std::mutex> lock(tpdo_time_mutex_);
            auto since_tpdo = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_tpdo_time_).count();
            if (since_tpdo > 500) connected_ = false;
        }

        std::this_thread::sleep_until(next_deadline);
        next_deadline += period;
    }
}

void MotorDriver::handleTPDO(const CanFrame& frame) {
    // TPDO1: StatusWord(16) + PositionActual(32) + ErrorCode(16)
    if (frame.id == static_cast<uint32_t>(0x180 + config_.node_id)) {
        if (frame.dlc < 8) return;

        uint16_t status_word;
        int32_t position_actual;
        uint16_t error_code;
        std::memcpy(&status_word, &frame.data[0], 2);
        std::memcpy(&position_actual, &frame.data[2], 4);
        std::memcpy(&error_code, &frame.data[6], 2);

        MotorStatus new_status;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            new_status = status_;
        }
        new_status.status_word = status_word;
        new_status.position_pulses = position_actual;
        new_status.error_code = error_code;
        new_status.is_enabled = (status_word & 0x04) != 0;
        new_status.target_reached = (status_word & 0x0400) != 0;
        new_status.is_fault = (status_word & 0x08) != 0;
        new_status.following_error = (status_word & 0x2000) != 0;

        // bit12 (Setpoint Acknowledge) — 驱动器接受新目标后置位,收到后清除 bit4
        if (status_word & 0x1000) {
            pdo_bit4_remaining_.store(0);  // 驱动器已确认,停止发 bit4
        }

        // 停止检测:电机速度为0但未到达目标且无故障 — 打印诊断信息(每秒最多1次)
        static auto last_stop_log = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        if (new_status.speed_01rpm == 0 && !new_status.target_reached &&
            !new_status.is_fault && last_target_pulses_ != 0 &&
            std::abs(position_actual - last_target_pulses_) > config_.mm_to_pulses) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stop_log).count() >= 1) {
                last_stop_log = now;
                int32_t remaining = (last_target_pulses_ - position_actual) / config_.mm_to_pulses;
                std::cerr << "[Motor] STOPPED: pos=" << position_actual / config_.mm_to_pulses
                          << "mm, target=" << last_target_pulses_ / config_.mm_to_pulses
                          << "mm, remaining=" << remaining << "mm"
                          << ", SW=0x" << std::hex << status_word << std::dec
                          << ", follow_err=" << new_status.following_error
                          << ", enabled=" << new_status.is_enabled << std::endl;
            }
        }

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_ = new_status;
        }

        {
            std::lock_guard<std::mutex> lock(tpdo_time_mutex_);
            last_tpdo_time_ = std::chrono::steady_clock::now();
        }

        if (status_callback_) status_callback_(new_status);
        return;
    }

    // TPDO2: VelocityActual(32) + DigitalInputs(32)
    if (frame.id == static_cast<uint32_t>(0x280 + config_.node_id)) {
        if (frame.dlc < 4) return;
        int32_t velocity_actual;
        std::memcpy(&velocity_actual, &frame.data[0], 4);
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.speed_01rpm = velocity_actual;
            if (frame.dlc >= 8) {
                std::memcpy(&status_.digital_inputs, &frame.data[4], 4);
                status_.digital_inputs_valid = true;
            }
        }
        // 不触发 callback: TPDO1 会以 200Hz 触发 publishState,
        // speed 在下一个 TPDO1 周期即可发布,避免重复发布
    }
}

void MotorDriver::reconnectLoop() {
    // PDO 模式断电重连看门狗:定期检测 connected_ 状态,
    // controlLoop 检测到 TPDO 超时会置 connected_=false,此处触发重连
    bool was_connected = true;
    while (reconnect_running_) {
        if (!connected_) {
            if (was_connected) {
                std::cerr << "[Motor] PDO: connection lost, attempting reconnect..." << std::endl;
                was_connected = false;
            }
            checkConnection();
        } else {
            was_connected = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

bool MotorDriver::enable() {
    if (!connected_ && !pdo_mode_) return false;

    // 先执行 fault reset(bit7=1),清除可能的残留故障
    if (pdo_mode_ && control_running_) {
        pdo_control_word_ = 0x80;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x00;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x06;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x07;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x0F;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return true;
    }

    sendControlWord(0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sendControlWord(0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!sendControlWord(0x06)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!sendControlWord(0x07)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!sendControlWord(0x0F)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    updateStatus();
    if (status_.is_enabled) {
        std::cout << "[Motor] Enabled" << std::endl;
        return true;
    }
    std::cerr << "[Motor] Enable check failed" << std::endl;
    return false;
}

bool MotorDriver::disable() {
    if (pdo_mode_ && control_running_) {
        pdo_control_word_ = 0x07;
        return true;
    }
    if (!connected_) return false;
    return sendControlWord(0x07);
}

bool MotorDriver::setPosition(int position_mm, int speed_rpm) {
    if (position_mm < config_.min_position_mm || position_mm > config_.max_position_mm) {
        std::cerr << "[Motor] Position out of range: " << position_mm << " mm" << std::endl;
        return false;
    }

    // 确定有效速度: speed_rpm <= 0 表示使用当前速度(current_speed_rpm_)
    int effective_speed = (speed_rpm > 0) ? speed_rpm : current_speed_rpm_.load();
    if (effective_speed > config_.max_speed_rpm) effective_speed = config_.max_speed_rpm;
    if (effective_speed < config_.min_speed_rpm) effective_speed = config_.min_speed_rpm;

    int32_t target_pulses = position_mm * config_.mm_to_pulses;

    if (pdo_mode_) {
        if (!connected_) {
            std::cerr << "[Motor] Not connected, cannot set position" << std::endl;
            return false;
        }
        // 如果控制循环未运行,重启 PDO
        if (!control_running_) {
            if (!writeSDO(OD_MODES_OP, 0, 1, 1)) {
                return setPositionSDO(position_mm, effective_speed);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            sendControlWord(0x0F);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            uint32_t pos;
            if (readSDO(OD_POSITION_ACTUAL, 0, pos)) {
                pdo_target_position_ = static_cast<int32_t>(pos);
            }
            pdo_control_word_ = 0x0F;
            startPDOMode();
        }

        // 始终通过 SDO 写入 profile velocity(PP 触发前写入,不干扰运动中 PDO)
        uint32_t pv = static_cast<uint32_t>(effective_speed * 10);
        bool pv_ok = writeSDO(OD_PROFILE_VELOCITY, 0, pv, 4);
        current_speed_rpm_ = effective_speed;
        if (!pv_ok) {
            std::cerr << "[Motor] WARNING: Failed to write profile velocity!" << std::endl;
        } else {
            // 读回验证(确认驱动器实际接受了该速度)
            uint32_t pv_read = 0;
            if (readSDO(OD_PROFILE_VELOCITY, 0, pv_read)) {
                int read_rpm = static_cast<int>(pv_read / 10);
                std::cout << "[Motor] Profile velocity: requested=" << effective_speed
                          << " rpm, readback=" << read_rpm << " rpm" << std::endl;
                if (read_rpm != effective_speed) {
                    std::cerr << "[Motor] WARNING: Speed readback mismatch! Drive may reject this speed." << std::endl;
                }
            }
        }

        // 等待驱动器处理完 SDO 并恢复 PDO 处理
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 更新目标位置并触发 bit4
        pdo_control_word_ = 0x0F;  // clear halt before accepting a new move
        pdo_target_position_ = target_pulses;
        last_target_pulses_ = target_pulses;
        pdo_trigger_pending_ = true;  // 下一个控制周期发 0x1F

        std::cout << "[Motor] PDO target: " << position_mm << " mm ("
                  << target_pulses << " pulses) at " << effective_speed << " rpm" << std::endl;
        return true;
    }

    return setPositionSDO(position_mm, effective_speed);
}

bool MotorDriver::stopMotion() {
    if (!connected_ && !pdo_mode_) return false;

    MotorStatus snapshot;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        snapshot = status_;
    }

    if (pdo_mode_ && control_running_) {
        pdo_target_position_ = snapshot.position_pulses;
        last_target_pulses_ = snapshot.position_pulses;
        pdo_trigger_pending_ = false;
        pdo_bit4_remaining_.store(0);
        pdo_control_word_ = 0x010F;  // CiA 402 halt, keep operation enabled
        std::cout << "[Motor] Motion halted at "
                  << snapshot.position_pulses / config_.mm_to_pulses << " mm" << std::endl;
        return true;
    }

    if (!connected_) return false;
    bool ok = sendControlWord(0x010F);
    if (ok) {
        std::cout << "[Motor] Motion halt requested" << std::endl;
    }
    return ok;
}

bool MotorDriver::setPositionSDO(int position_mm, int speed_rpm) {
    if (!connected_) return false;

    int32_t pulses = position_mm * config_.mm_to_pulses;

    if (!writeSDO(OD_MODES_OP, 0, 1, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sendControlWord(0x0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // speed_rpm 已由 setPosition() 计算为有效速度(clamped)
    uint32_t pv = static_cast<uint32_t>(speed_rpm * 10);
    if (!writeSDO(OD_PROFILE_VELOCITY, 0, pv, 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (!writeSDO(OD_TARGET_POSITION, 0, static_cast<uint32_t>(pulses), 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // PP 触发:bit4 上升沿 → 清除
    if (!writeSDO(OD_CONTROLWORD, 0, 0x1F, 2)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!writeSDO(OD_CONTROLWORD, 0, 0x0F, 2)) return false;

    std::cout << "[Motor] Moving to " << position_mm << " mm ("
              << pulses << " pulses) at " << speed_rpm << " rpm" << std::endl;
    return true;
}

bool MotorDriver::setSpeed(int speed_rpm) {
    if (!connected_) return false;

    // 速度限幅
    if (speed_rpm > config_.max_speed_rpm) {
        std::cerr << "[Motor] Speed clamped: " << speed_rpm << " -> " << config_.max_speed_rpm << " rpm" << std::endl;
        speed_rpm = config_.max_speed_rpm;
    } else if (speed_rpm < config_.min_speed_rpm) {
        std::cerr << "[Motor] Speed clamped: " << speed_rpm << " -> " << config_.min_speed_rpm << " rpm" << std::endl;
        speed_rpm = config_.min_speed_rpm;
    }

    // 只更新记录,不写 SDO — 实际 profile velocity 由 setPosition() 在 PP 触发前写入
    // 这样避免运动中 SDO 写入干扰 PDO 通信
    current_speed_rpm_ = speed_rpm;
    std::cout << "[Motor] Speed set: " << speed_rpm << " rpm (effective on next move)" << std::endl;
    return true;
}

bool MotorDriver::setZero() {
    if (!connected_) return false;

    bool was_pdo = false;
    if (pdo_mode_ && control_running_) {
        stopPDOMode();
        was_pdo = true;
    }

    // homing method 35 = 不回零(设置当前位置为零点)
    if (!writeSDO(OD_HOMING_METHOD, 0, 35, 1)) {
        std::cerr << "[Motor] Failed to set homing method 35" << std::endl;
        if (was_pdo) restartPDO();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!writeSDO(OD_MODES_OP, 0, 6, 1)) {
        std::cerr << "[Motor] Failed to switch to homing mode" << std::endl;
        if (was_pdo) restartPDO();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!sendControlWord(0x1F)) {
        if (was_pdo) restartPDO();
        return false;
    }

    // 等待 homing 完成 (bit12)
    for (int i = 0; i < 20; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        uint16_t sw;
        if (readSDO16(OD_STATUSWORD, 0, sw)) {
            if (sw & 0x1000) break;
            if (sw & 0x2000) {
                std::cerr << "[Motor] Homing error" << std::endl;
                sendControlWord(0x0F);
                writeSDO(OD_MODES_OP, 0, 1, 1);
                if (was_pdo) restartPDO();
                return false;
            }
        }
    }

    sendControlWord(0x0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (was_pdo) {
        restartPDO();
    } else {
        if (!writeSDO(OD_MODES_OP, 0, 1, 1)) {
            std::cerr << "[Motor] Failed to restore PP mode" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sendControlWord(0x0F);
    }

    // 更新软件位置限位(以新零位为基准)
    int32_t min_pulses = config_.min_position_mm * config_.mm_to_pulses;
    int32_t max_pulses = config_.max_position_mm * config_.mm_to_pulses;
    writeSDO(OD_POSITION_LIMIT, 1, static_cast<uint32_t>(min_pulses), 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    writeSDO(OD_POSITION_LIMIT, 2, static_cast<uint32_t>(max_pulses), 4);

    std::cout << "[Motor] Zero position set" << std::endl;
    return true;
}

void MotorDriver::restartPDO() {
    if (!writeSDO(OD_MODES_OP, 0, 1, 1)) {
        std::cerr << "[Motor] Failed to restore PP mode" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sendControlWord(0x0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    uint32_t pos;
    if (readSDO(OD_POSITION_ACTUAL, 0, pos)) {
        pdo_target_position_ = static_cast<int32_t>(pos);
    }
    pdo_control_word_ = 0x0F;
    pdo_trigger_pending_ = false;
    {
        std::lock_guard<std::mutex> lock(tpdo_time_mutex_);
        last_tpdo_time_ = std::chrono::steady_clock::now();
    }
    startPDOMode();
}

bool MotorDriver::clearError() {
    if (!connected_ && !pdo_mode_) return false;
    std::cout << "[Motor] Clearing error" << std::endl;

    if (pdo_mode_ && control_running_) {
        pdo_control_word_ = 0x80;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x06;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x07;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pdo_control_word_ = 0x0F;
        return true;
    }

    if (!sendControlWord(0x80)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sendControlWord(0x06);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sendControlWord(0x07);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return sendControlWord(0x0F);
}

bool MotorDriver::resetDrive() {
    if (!connected_) return false;
    std::cout << "[Motor] Resetting drive" << std::endl;
    return reconfigureDrive();
}

void MotorDriver::statusUpdateLoop() {
    while (running_) {
        updateStatus();
        if (status_callback_) {
            MotorStatus snapshot;
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                snapshot = status_;
            }
            status_callback_(snapshot);
        }
        checkConnection();
        int interval_ms = 1000 / config_.status_update_hz;
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
}

bool MotorDriver::updateStatus() {
    if (!connected_) return false;

    int failures = 0;
    MotorStatus new_status = status_;

    uint16_t status_word;
    if (readSDO16(OD_STATUSWORD, 0, status_word)) {
        new_status.status_word = status_word;
        new_status.is_enabled = (status_word & 0x04) != 0;
        new_status.target_reached = (status_word & 0x0400) != 0;
        new_status.is_fault = (status_word & 0x08) != 0;
        new_status.following_error = (status_word & 0x2000) != 0;
    } else {
        failures++;
    }

    uint16_t error_code;
    if (readSDO16(OD_ERROR_CODE, 0, error_code)) {
        new_status.error_code = error_code;
    } else {
        failures++;
    }

    uint32_t pos_pulses;
    if (readSDO(OD_POSITION_ACTUAL, 0, pos_pulses)) {
        new_status.position_pulses = static_cast<int32_t>(pos_pulses);
    } else {
        failures++;
    }

    uint32_t speed_01rpm;
    if (readSDO(OD_VELOCITY_ACTUAL, 0, speed_01rpm)) {
        new_status.speed_01rpm = static_cast<int32_t>(speed_01rpm);
    } else {
        failures++;
    }

    uint32_t digital_inputs;
    if (readSDO(OD_DIGITAL_INPUTS, 0, digital_inputs)) {
        new_status.digital_inputs = digital_inputs;
        new_status.digital_inputs_valid = true;
    } else {
        new_status.digital_inputs_valid = false;
        failures++;
    }

    if (failures >= 4) {
        std::cerr << "[Motor] Drive not responding (" << failures
                  << "/5 SDO reads failed)" << std::endl;
        connected_ = false;
    }

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_ = new_status;
    }
    return failures == 0;
}

MotorStatus MotorDriver::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

int MotorDriver::getPositionMM() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_.position_pulses / config_.mm_to_pulses;
}

int MotorDriver::getSpeedRPM() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_.speed_01rpm / 10;
}

void MotorDriver::checkConnection() {
    if (!can_iface_.is_connected()) {
        connected_ = false;
        if (tryReconnect()) std::cout << "[Motor] CAN reconnected" << std::endl;
    } else if (!connected_) {
        if (tryReconnect()) std::cout << "[Motor] Drive reconnected" << std::endl;
    }
}

bool MotorDriver::tryReconnect() {
    static auto last_attempt = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attempt);
    if (elapsed.count() < config_.reconnect_timeout_ms) return false;
    last_attempt = now;

    bool was_pdo = pdo_mode_ && control_running_;
    if (was_pdo) stopPDOMode();

    if (!can_iface_.is_connected()) {
        if (!can_iface_.init()) return false;
    }

    // 临时置位以允许 SDO 探测(连接状态由 SDO 响应决定)
    connected_ = true;

    uint16_t status_word;
    if (!readSDO16(OD_STATUSWORD, 0, status_word)) {
        connected_ = false;  // SDO 失败,电机仍未响应
        return false;
    }

    std::cout << "[Motor] Drive responding, reconfiguring..." << std::endl;

    if (!reconfigureDrive()) {
        std::cerr << "[Motor] Reconfigure failed" << std::endl;
        connected_ = false;
        return false;
    }

    if (config_.pdo_enabled) {
        if (configurePDO()) {
            pdo_mode_ = true;
        } else {
            pdo_mode_ = false;
        }
    }

    if (!enable()) {
        std::cerr << "[Motor] Re-enable failed" << std::endl;
        connected_ = false;
        return false;
    }

    if (pdo_mode_) {
        uint32_t pos;
        if (readSDO(OD_POSITION_ACTUAL, 0, pos)) {
            pdo_target_position_ = static_cast<int32_t>(pos);
        }
        pdo_control_word_ = 0x0F;
        pdo_trigger_pending_ = false;
        {
            std::lock_guard<std::mutex> lock(tpdo_time_mutex_);
            last_tpdo_time_ = std::chrono::steady_clock::now();
        }
        startPDOMode();
    } else if (!status_thread_.joinable()) {
        status_thread_ = std::thread(&MotorDriver::statusUpdateLoop, this);
    }

    connected_ = true;
    std::cout << "[Motor] Reconnected successfully" << std::endl;
    return true;
}

void MotorDriver::setStatusCallback(std::function<void(const MotorStatus&)> callback) {
    status_callback_ = callback;
}

} // namespace lift_controller
