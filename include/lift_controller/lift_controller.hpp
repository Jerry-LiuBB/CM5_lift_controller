#ifndef LIFT_CONTROLLER_LIFT_CONTROLLER_HPP_
#define LIFT_CONTROLLER_LIFT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "lift_controller/motor_driver.hpp"

namespace lift_controller {

class LiftController : public rclcpp::Node {
public:
    LiftController();
    ~LiftController();

private:
    void loadConfig();
    void initDriver();
    void publishState();
    void statusCallback(const MotorStatus& status);

    // Topic callbacks
    void setPositionCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void setPositionWithSpeedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void setZeroCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void clearErrorCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void updateDriveLimitState(const MotorStatus& status);
    bool rejectPositionIfBlockedByLimit(const char* command_name, int target_pos_mm);
    bool rejectCommandIfSafetyFault(const char* command_name);
    void setLimitStopState(bool active, const std::string& reason);
    void checkLimitStatusTimeout();

    // Components
    std::unique_ptr<CanInterface> can_interface_;
    std::unique_ptr<MotorDriver> motor_driver_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_percent_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_code_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr following_error_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr version_pub_;  // 驱动器版本信息(latched)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr digital_inputs_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_with_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_error_sub_;

    rclcpp::TimerBase::SharedPtr state_timer_;

    MotorStatus current_status_;
    std::mutex status_mutex_;
    mutable std::mutex limit_mutex_;
    bool limit_safety_enabled_ = true;
    double limit_status_timeout_sec_ = 0.5;
    std::string limit_1_direction_ = "min";
    std::string limit_2_direction_ = "max";
    int limit_1_block_direction_ = -1;
    int limit_2_block_direction_ = 1;
    uint32_t limit_1_input_mask_ = 0x1;
    uint32_t limit_2_input_mask_ = 0x2;
    bool limit_inputs_active_high_ = true;
    bool limit_stop_active_ = false;
    bool limit_safety_fault_active_ = false;
    bool have_drive_limit_status_ = false;
    bool limit_1_active_ = false;
    bool limit_2_active_ = false;
    rclcpp::Time last_drive_limit_status_time_;
    std::string limit_stop_reason_;

    struct Config {
        std::string can_interface = "can1";
        int can_bitrate = 1000000;  // 期望的 CAN 波特率(实际由 ip link 设置,此处仅作配置/提示)
        int node_id = 1;
        int mm_to_pulses = 1000;
        int max_position_mm = 500;
        int min_position_mm = 0;
        int default_speed_rpm = 100;
        int max_speed_rpm = 200;
        int min_speed_rpm = 0;
        int accel_time_ms = 500;
        int decel_time_ms = 500;
        int following_error_pulses = 100000;  // 跟随误差窗口(脉冲数),0=用驱动器内值
        int max_motor_speed_rpm = 0;          // 电机最大转速(rpm),0=用驱动器内值
        int status_update_hz = 10;
        int reconnect_timeout_ms = 1000;
        bool pdo_enabled = true;       // 是否启用 PDO 模式
        int pdo_frequency_hz = 200;    // PDO 控制循环频率
    } config_;
};

} // namespace lift_controller

#endif
