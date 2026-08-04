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

namespace lift_controller {

struct MotorStatus {
    uint16_t status_word;
    uint16_t error_code;
    int32_t position_pulses;
    int32_t speed_01rpm;
    bool is_enabled;
    bool target_reached;
    bool is_fault;
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
    bool setZero();
    bool clearError();
    bool resetDrive();

    MotorStatus getStatus() const;
    bool updateStatus();
    int getPositionMM() const;
    int getSpeedRPM() const;

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
    std::atomic<bool> pdo_trigger_pending_;  // PP 模式 bit4 触发标志

    // 重连看门狗(PDO 模式下检测断电并自动重连)
    std::atomic<bool> reconnect_running_;
    std::thread reconnect_thread_;

    std::chrono::steady_clock::time_point last_tpdo_time_;
    mutable std::mutex tpdo_time_mutex_;
};

} // namespace lift_controller

#endif
