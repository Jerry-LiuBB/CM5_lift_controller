#ifndef LIFT_CONTROLLER_MOTOR_DRIVER_HPP_
#define LIFT_CONTROLLER_MOTOR_DRIVER_HPP_

#include "lift_controller/can_interface.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>
#include <chrono>
#include <iostream>
#include <cstring>
#include <string>

namespace lift_controller {

struct MotorStatus {
    uint16_t status_word;
    uint16_t error_code;
    int32_t position_pulses;
    int32_t speed_01rpm;
    uint16_t limit_status;
    bool is_enabled;
    bool target_reached;
    bool is_fault;
    bool following_error;  // bit13: 位置跟随误差超限
    bool limit_status_valid;
};

struct MotorConfig {
    int node_id = 1;
    int mm_to_pulses = 1000;
    int max_position_mm = 500;
    int min_position_mm = 0;
    int default_speed_rpm = 100;
    int max_speed_rpm = 200;
    int min_speed_rpm = 0;
    int accel_time_ms = 500;
    int decel_time_ms = 500;
    int following_error_pulses = 100000;  // 跟随误差窗口(脉冲数),0=不限制
    int max_motor_speed_rpm = 0;          // 电机最大转速(rpm),0=不设置(用驱动器内值)
    int reconnect_timeout_ms = 1000;
    int status_update_hz = 10;
    bool pdo_enabled = true;
    int pdo_frequency_hz = 200;
};

class MotorDriver {
public:
    explicit MotorDriver(CanInterface& can_iface);
    ~MotorDriver();

    bool init(const MotorConfig& config);
    void shutdown();

    bool enable();
    bool disable();
    bool setSpeed(int speed_rpm);
    bool setPosition(int position_mm, int speed_rpm = 0);
    bool stopMotion();
    bool setZero();
    bool clearError();
    bool resetDrive();

    MotorStatus getStatus() const;
    bool updateStatus();
    int getPositionMM() const;
    int getSpeedRPM() const;
    int getCurrentSpeedRPM() const { return current_speed_rpm_.load(); }
    std::string getVersion() const { return version_info_; }

    void setConfig(const MotorConfig& config);
    const MotorConfig& getConfig() const { return config_; }

    void setStatusCallback(std::function<void(const MotorStatus&)> callback);
    bool tryReconnect();

    bool isPDOMode() const { return pdo_mode_; }

private:
    bool writeSDO(uint16_t index, uint8_t subindex, uint32_t data, uint8_t length);
    bool readSDO(uint16_t index, uint8_t subindex, uint32_t& data);
    bool readSDO16(uint16_t index, uint8_t subindex, uint16_t& data);
    bool sendControlWord(uint16_t value);
    bool sendNMT(uint8_t command);

    void statusUpdateLoop();
    void checkConnection();
    bool reconfigureDrive();
    bool setPositionSDO(int position_mm, int speed_rpm);
    void readVersionInfo();  // 读取驱动器版本信息(0x1018)

    // PDO 模式
    bool configurePDO();
    void controlLoop();
    void handleTPDO(const CanFrame& frame);
    void startPDOMode();
    void stopPDOMode();
    void restartPDO();
    void reconnectLoop();  // PDO 模式断电重连看门狗

    CanInterface& can_iface_;
    MotorConfig config_;
    MotorStatus status_;
    mutable std::mutex status_mutex_;
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    std::thread status_thread_;
    std::function<void(const MotorStatus&)> status_callback_;

    // PDO 模式状态
    bool pdo_mode_;
    std::atomic<bool> control_running_;
    std::thread control_thread_;
    std::atomic<uint16_t> pdo_control_word_;
    std::atomic<int32_t> pdo_target_position_;
    int32_t last_target_pulses_ = 0;  // 上次命令的目标位置(用于停止检测)
    std::atomic<bool> pdo_trigger_pending_;  // PP 模式 bit4 触发标志
    std::atomic<int> pdo_bit4_remaining_{0};  // bit4 保持周期数(等驱动器回 bit12)
    std::atomic<int> current_speed_rpm_;     // 当前 PP 模式运动速度(rpm)

    // 重连看门狗(PDO 模式下检测断电并自动重连)
    std::atomic<bool> reconnect_running_;
    std::thread reconnect_thread_;

    std::chrono::steady_clock::time_point last_tpdo_time_;
    mutable std::mutex tpdo_time_mutex_;

    std::string version_info_;  // 驱动器版本信息
};

} // namespace lift_controller

#endif
