#ifndef LIFT_CONTROLLER_LIFT_CONTROLLER_HPP_
#define LIFT_CONTROLLER_LIFT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
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

    // Components
    std::unique_ptr<CanInterface> can_interface_;
    std::unique_ptr<MotorDriver> motor_driver_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_code_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;

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

    struct Config {
        std::string can_interface = "can0";
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
        int status_update_hz = 10;
        int reconnect_timeout_ms = 1000;
        bool pdo_enabled = true;       // 是否启用 PDO 模式
        int pdo_frequency_hz = 200;    // PDO 控制循环频率
    } config_;
};

} // namespace lift_controller

#endif