#include "lift_controller/lift_controller.hpp"

namespace lift_controller {

LiftController::LiftController() : Node("lift_controller") {
    loadConfig();

    RCLCPP_INFO(this->get_logger(),
                "Expecting %s @ %d bps — run: sudo bash setup_can.sh %s %d",
                config_.can_interface.c_str(), config_.can_bitrate,
                config_.can_interface.c_str(), config_.can_bitrate);

    can_interface_ = std::make_unique<CanInterface>(config_.can_interface);
    if (!can_interface_->init()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
        return;
    }

    // 先创建所有 publisher(因 TPDO 回调会触发 publish,必须在 motor init 之前就绪)
    position_pub_ = this->create_publisher<std_msgs::msg::Float64>("lift/position", 10);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("lift/speed", 10);
    speed_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("lift/speed_rpm", 10);
    speed_percent_pub_ = this->create_publisher<std_msgs::msg::Float64>("lift/speed_percent", 10);
    error_code_pub_ = this->create_publisher<std_msgs::msg::Int32>("lift/error_code", 10);
    enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("lift/enabled", 10);
    target_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("lift/target_reached", 10);
    fault_pub_ = this->create_publisher<std_msgs::msg::Bool>("lift/fault", 10);
    following_error_pub_ = this->create_publisher<std_msgs::msg::Bool>("lift/following_error", 10);

    // Subscribers
    target_pos_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "lift/target_position", 10,
        std::bind(&LiftController::setPositionCallback, this, std::placeholders::_1));

    target_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "lift/target_speed", 10,
        std::bind(&LiftController::setSpeedCallback, this, std::placeholders::_1));

    pos_with_speed_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "lift/target_position_with_speed", 10,
        std::bind(&LiftController::setPositionWithSpeedCallback, this, std::placeholders::_1));

    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "lift/enable_cmd", 10,
        std::bind(&LiftController::enableCallback, this, std::placeholders::_1));

    zero_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "lift/zero_cmd", 10,
        std::bind(&LiftController::setZeroCallback, this, std::placeholders::_1));

    clear_error_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "lift/clear_error_cmd", 10,
        std::bind(&LiftController::clearErrorCallback, this, std::placeholders::_1));

    // 初始化电机驱动器(init 内会启动 PDO 控制循环,TPDO 回调将触发 publish)
    motor_driver_ = std::make_unique<MotorDriver>(*can_interface_);
    MotorConfig motor_config;
    motor_config.node_id = config_.node_id;
    motor_config.mm_to_pulses = config_.mm_to_pulses;
    motor_config.max_position_mm = config_.max_position_mm;
    motor_config.min_position_mm = config_.min_position_mm;
    motor_config.default_speed_rpm = config_.default_speed_rpm;
    motor_config.max_speed_rpm = config_.max_speed_rpm;
    motor_config.min_speed_rpm = config_.min_speed_rpm;
    motor_config.accel_time_ms = config_.accel_time_ms;
    motor_config.decel_time_ms = config_.decel_time_ms;
    motor_config.following_error_pulses = config_.following_error_pulses;
    motor_config.max_motor_speed_rpm = config_.max_motor_speed_rpm;
    motor_config.reconnect_timeout_ms = config_.reconnect_timeout_ms;
    motor_config.status_update_hz = config_.status_update_hz;
    motor_config.pdo_enabled = config_.pdo_enabled;
    motor_config.pdo_frequency_hz = config_.pdo_frequency_hz;

    motor_driver_->setStatusCallback(
        std::bind(&LiftController::statusCallback, this, std::placeholders::_1));

    if (!motor_driver_->init(motor_config)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize motor driver");
        return;
    }

    // 启动时使能电机(PDO 模式下 init 内已使能,此处为 SDO 回退模式的保险)
    motor_driver_->enable();

    // State timer: SDO 回退模式的状态发布;PDO 模式下由 TPDO 回调驱动(定时器跳过)
    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / config_.status_update_hz),
        [this]() {
            if (!motor_driver_ || !motor_driver_->isPDOMode()) {
                publishState();
            }
        });

    RCLCPP_INFO(this->get_logger(), "Lift Controller started (%s mode)",
                motor_driver_->isPDOMode() ? "PDO" : "SDO");
}

LiftController::~LiftController() {
    if (motor_driver_) {
        motor_driver_->disable();
    }
}

void LiftController::loadConfig() {
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("can_bitrate", 1000000);
    this->declare_parameter("node_id", 1);
    this->declare_parameter("mm_to_pulses", 1000);
    this->declare_parameter("max_position_mm", 500);
    this->declare_parameter("min_position_mm", 0);
    this->declare_parameter("default_speed_rpm", 100);
    this->declare_parameter("max_speed_rpm", 200);
    this->declare_parameter("min_speed_rpm", 0);
    this->declare_parameter("accel_time_ms", 500);
    this->declare_parameter("decel_time_ms", 500);
    this->declare_parameter("following_error_pulses", 100000);
    this->declare_parameter("max_motor_speed_rpm", 0);
    this->declare_parameter("status_update_hz", 10);
    this->declare_parameter("reconnect_timeout_ms", 1000);
    this->declare_parameter("pdo_enabled", true);
    this->declare_parameter("pdo_frequency_hz", 200);

    this->get_parameter("can_interface", config_.can_interface);
    this->get_parameter("can_bitrate", config_.can_bitrate);
    this->get_parameter("node_id", config_.node_id);
    this->get_parameter("mm_to_pulses", config_.mm_to_pulses);
    this->get_parameter("max_position_mm", config_.max_position_mm);
    this->get_parameter("min_position_mm", config_.min_position_mm);
    this->get_parameter("default_speed_rpm", config_.default_speed_rpm);
    this->get_parameter("max_speed_rpm", config_.max_speed_rpm);
    this->get_parameter("min_speed_rpm", config_.min_speed_rpm);
    this->get_parameter("accel_time_ms", config_.accel_time_ms);
    this->get_parameter("decel_time_ms", config_.decel_time_ms);
    this->get_parameter("following_error_pulses", config_.following_error_pulses);
    this->get_parameter("max_motor_speed_rpm", config_.max_motor_speed_rpm);
    this->get_parameter("status_update_hz", config_.status_update_hz);
    this->get_parameter("reconnect_timeout_ms", config_.reconnect_timeout_ms);
    this->get_parameter("pdo_enabled", config_.pdo_enabled);
    this->get_parameter("pdo_frequency_hz", config_.pdo_frequency_hz);
}

void LiftController::statusCallback(const MotorStatus& status) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = status;
    }
    // PDO 模式:TPDO 驱动发布(200Hz),直接调用 publishState
    // SDO 模式:由 state_timer 定期发布
    if (motor_driver_ && motor_driver_->isPDOMode()) {
        publishState();
    }
}

void LiftController::publishState() {
    std::lock_guard<std::mutex> lock(status_mutex_);

    auto pos_msg = std_msgs::msg::Float64();
    pos_msg.data = static_cast<double>(current_status_.position_pulses) / config_.mm_to_pulses;
    position_pub_->publish(pos_msg);

    auto speed_msg = std_msgs::msg::Float64();
    speed_msg.data = static_cast<double>(current_status_.speed_01rpm) / 10.0;
    speed_pub_->publish(speed_msg);

    // 当前设定速度(rpm)及百分比(相对 max_speed_rpm)
    auto speed_rpm_msg = std_msgs::msg::Float64();
    auto speed_percent_msg = std_msgs::msg::Float64();
    if (motor_driver_) {
        int current_rpm = motor_driver_->getCurrentSpeedRPM();
        speed_rpm_msg.data = static_cast<double>(current_rpm);
        if (config_.max_speed_rpm > 0) {
            speed_percent_msg.data =
                (static_cast<double>(current_rpm) / config_.max_speed_rpm) * 100.0;
        } else {
            speed_percent_msg.data = 0.0;
        }
    }
    speed_rpm_pub_->publish(speed_rpm_msg);
    speed_percent_pub_->publish(speed_percent_msg);

    auto error_msg = std_msgs::msg::Int32();
    error_msg.data = current_status_.error_code;
    error_code_pub_->publish(error_msg);

    auto enabled_msg = std_msgs::msg::Bool();
    enabled_msg.data = current_status_.is_enabled;
    enabled_pub_->publish(enabled_msg);

    auto reached_msg = std_msgs::msg::Bool();
    reached_msg.data = current_status_.target_reached;
    target_reached_pub_->publish(reached_msg);

    auto fault_msg = std_msgs::msg::Bool();
    fault_msg.data = current_status_.is_fault;
    fault_pub_->publish(fault_msg);

    auto follow_err_msg = std_msgs::msg::Bool();
    follow_err_msg.data = current_status_.following_error;
    following_error_pub_->publish(follow_err_msg);
}

void LiftController::setPositionCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    int pos_mm = static_cast<int>(msg->data);
    if (pos_mm < config_.min_position_mm || pos_mm > config_.max_position_mm) {
        RCLCPP_WARN(this->get_logger(), "Position %d mm out of range [%d, %d], ignored",
                    pos_mm, config_.min_position_mm, config_.max_position_mm);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Moving to position: %d mm", pos_mm);
    motor_driver_->setPosition(pos_mm);
}

void LiftController::setSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    int speed_rpm = static_cast<int>(msg->data);
    if (speed_rpm > config_.max_speed_rpm || speed_rpm < config_.min_speed_rpm) {
        RCLCPP_WARN(this->get_logger(), "Speed %d rpm out of range [%d, %d], clamped",
                    speed_rpm, config_.min_speed_rpm, config_.max_speed_rpm);
    }
    RCLCPP_INFO(this->get_logger(), "Setting speed: %d rpm", speed_rpm);
    motor_driver_->setSpeed(speed_rpm);
}

void LiftController::setPositionWithSpeedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "target_position_with_speed needs [position_mm, speed_rpm]");
        return;
    }
    int pos_mm = static_cast<int>(msg->data[0]);
    int speed_rpm = static_cast<int>(msg->data[1]);
    if (pos_mm < config_.min_position_mm || pos_mm > config_.max_position_mm) {
        RCLCPP_WARN(this->get_logger(), "Position %d mm out of range [%d, %d], ignored",
                    pos_mm, config_.min_position_mm, config_.max_position_mm);
        return;
    }
    if (speed_rpm > config_.max_speed_rpm || speed_rpm < config_.min_speed_rpm) {
        RCLCPP_WARN(this->get_logger(), "Speed %d rpm out of range [%d, %d], clamped",
                    speed_rpm, config_.min_speed_rpm, config_.max_speed_rpm);
    }
    RCLCPP_INFO(this->get_logger(), "Moving to position: %d mm at %d rpm", pos_mm, speed_rpm);
    motor_driver_->setPosition(pos_mm, speed_rpm);
}

void LiftController::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        motor_driver_->enable();
    } else {
        motor_driver_->disable();
    }
}

void LiftController::setZeroCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        motor_driver_->setZero();
    }
}

void LiftController::clearErrorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        motor_driver_->clearError();
    }
}

} // namespace lift_controller
