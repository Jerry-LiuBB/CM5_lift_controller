#include "lift_controller/lift_controller.hpp"

namespace lift_controller {
namespace {

int parseLimitDirection(const std::string& direction, const std::string& name,
                        const rclcpp::Logger& logger) {
    if (direction == "min" || direction == "lower" || direction == "negative") {
        return -1;
    }
    if (direction == "max" || direction == "upper" || direction == "positive") {
        return 1;
    }
    if (direction == "none" || direction == "disabled") {
        return 0;
    }
    RCLCPP_WARN(logger,
                "%s direction '%s' is invalid; expected min/max/none, using none",
                name.c_str(), direction.c_str());
    return 0;
}

const char* directionName(int direction) {
    if (direction < 0) return "min";
    if (direction > 0) return "max";
    return "none";
}

}  // namespace

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
    limit_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("lift/limit_stop", 10);

    // 版本信息使用 latched QoS — 后订阅的节点也能立即收到
    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local();
    version_pub_ = this->create_publisher<std_msgs::msg::String>("lift/version_info", latched_qos);

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

    if (limit_safety_enabled_) {
        power_board_status_sub_ = this->create_subscription<power_board_msgs::msg::PowerBoardStatus>(
            power_board_status_topic_, 10,
            std::bind(&LiftController::powerBoardStatusCallback, this, std::placeholders::_1));
    }

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

    // 发布驱动器版本信息(latched)
    auto ver_msg = std_msgs::msg::String();
    ver_msg.data = motor_driver_->getVersion();
    version_pub_->publish(ver_msg);
    RCLCPP_INFO(this->get_logger(), "Drive version: %s", ver_msg.data.c_str());

    // State timer: SDO 回退模式的状态发布;PDO 模式下由 TPDO 回调驱动(定时器跳过)
    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / config_.status_update_hz),
        [this]() {
            checkLimitStatusTimeout();
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
    this->declare_parameter("can_interface", "can1");
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
    this->declare_parameter("limit_safety_enabled", true);
    this->declare_parameter("power_board_status_topic", "/power_board/status");
    this->declare_parameter("limit_1_direction", "min");
    this->declare_parameter("limit_2_direction", "max");
    this->declare_parameter("limit_stop_on_power_board_fault", true);
    this->declare_parameter("limit_status_timeout_sec", 0.5);

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
    this->get_parameter("limit_safety_enabled", limit_safety_enabled_);
    this->get_parameter("power_board_status_topic", power_board_status_topic_);
    this->get_parameter("limit_1_direction", limit_1_direction_);
    this->get_parameter("limit_2_direction", limit_2_direction_);
    this->get_parameter("limit_stop_on_power_board_fault", limit_stop_on_power_board_fault_);
    this->get_parameter("limit_status_timeout_sec", limit_status_timeout_sec_);

    limit_1_block_direction_ =
        parseLimitDirection(limit_1_direction_, "limit_1_direction", this->get_logger());
    limit_2_block_direction_ =
        parseLimitDirection(limit_2_direction_, "limit_2_direction", this->get_logger());
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

void LiftController::powerBoardStatusCallback(
    const power_board_msgs::msg::PowerBoardStatus::SharedPtr msg) {
    if (!limit_safety_enabled_) return;

    const bool limit_1_enabled = limit_1_block_direction_ != 0;
    const bool limit_2_enabled = limit_2_block_direction_ != 0;
    const bool limit_1_invalid = limit_1_enabled && !msg->limit_1_valid;
    const bool limit_2_invalid = limit_2_enabled && !msg->limit_2_valid;
    const bool limit_1_triggered = msg->limit_1_triggered && limit_1_enabled;
    const bool limit_2_triggered = msg->limit_2_triggered && limit_2_enabled;

    bool active = limit_1_triggered || limit_2_triggered;
    std::string reason;
    if (limit_1_triggered && limit_2_triggered) {
        reason = "LIMIT_1 and LIMIT_2 triggered";
    } else if (limit_1_triggered) {
        reason = "LIMIT_1 triggered";
    } else if (limit_2_triggered) {
        reason = "LIMIT_2 triggered";
    }

    if (limit_1_invalid || limit_2_invalid) {
        active = true;
        if (limit_1_invalid && limit_2_invalid) {
            reason = "LIMIT_1 and LIMIT_2 inputs invalid";
        } else if (limit_1_invalid) {
            reason = "LIMIT_1 input invalid";
        } else {
            reason = "LIMIT_2 input invalid";
        }
    } else if (limit_stop_on_power_board_fault_) {
        if (!msg->communication_ok) {
            active = true;
            reason = "power board communication fault: " + msg->last_error;
        } else if (msg->acquisition_stalled) {
            active = true;
            reason = "power board acquisition stalled";
        }
    }

    {
        std::lock_guard<std::mutex> lock(limit_mutex_);
        have_power_board_status_ = true;
        last_power_board_status_time_ = this->now();
        limit_1_active_ = limit_1_triggered;
        limit_2_active_ = limit_2_triggered;
        limit_safety_fault_active_ =
            limit_1_invalid || limit_2_invalid ||
            (limit_stop_on_power_board_fault_ &&
             (!msg->communication_ok || msg->acquisition_stalled));
    }

    setLimitStopState(active, reason);
}

bool LiftController::rejectPositionIfBlockedByLimit(
    const char* command_name, int target_pos_mm) {
    std::lock_guard<std::mutex> lock(limit_mutex_);
    if (!limit_safety_enabled_) return false;

    if (limit_safety_fault_active_) {
        RCLCPP_WARN(this->get_logger(), "%s ignored: lift safety fault is active (%s)",
                    command_name, limit_stop_reason_.c_str());
        return true;
    }

    int current_pos_mm = 0;
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        current_pos_mm = current_status_.position_pulses / config_.mm_to_pulses;
    }

    int requested_direction = 0;
    if (target_pos_mm > current_pos_mm) {
        requested_direction = 1;
    } else if (target_pos_mm < current_pos_mm) {
        requested_direction = -1;
    }

    if (requested_direction == 0) return false;

    const bool limit_1_blocks =
        limit_1_active_ && limit_1_block_direction_ == requested_direction;
    const bool limit_2_blocks =
        limit_2_active_ && limit_2_block_direction_ == requested_direction;
    if (!limit_1_blocks && !limit_2_blocks) return false;

    std::string reason;
    if (limit_1_blocks && limit_2_blocks) {
        reason = "LIMIT_1 and LIMIT_2 block " + std::string(directionName(requested_direction)) +
                 " direction";
    } else if (limit_1_blocks) {
        reason = "LIMIT_1 blocks " + std::string(directionName(requested_direction)) +
                 " direction";
    } else {
        reason = "LIMIT_2 blocks " + std::string(directionName(requested_direction)) +
                 " direction";
    }
    RCLCPP_WARN(this->get_logger(),
                "%s ignored: %s (current=%d mm, target=%d mm)",
                command_name, reason.c_str(), current_pos_mm, target_pos_mm);
    return true;
}

bool LiftController::rejectCommandIfSafetyFault(const char* command_name) {
    std::lock_guard<std::mutex> lock(limit_mutex_);
    if (!limit_safety_enabled_ || !limit_safety_fault_active_) return false;
    RCLCPP_WARN(this->get_logger(), "%s ignored: lift safety fault is active (%s)",
                command_name, limit_stop_reason_.c_str());
    return true;
}

void LiftController::setLimitStopState(bool active, const std::string& reason) {
    if (!limit_safety_enabled_) return;

    bool rising_edge = false;
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(limit_mutex_);
        rising_edge = active && !limit_stop_active_;
        changed = active != limit_stop_active_;
        limit_stop_active_ = active;
        limit_stop_reason_ = active ? reason : "";
        if (!active) {
            limit_safety_fault_active_ = false;
        }
    }

    if (rising_edge) {
        RCLCPP_ERROR(this->get_logger(), "Lift limit stop active: %s", reason.c_str());
        if (motor_driver_) {
            motor_driver_->stopMotion();
        }
    } else if (changed) {
        RCLCPP_INFO(this->get_logger(), "Lift limit stop cleared");
    }

    if (changed && limit_stop_pub_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = active;
        limit_stop_pub_->publish(msg);
    }
}

void LiftController::checkLimitStatusTimeout() {
    if (!limit_safety_enabled_ || limit_status_timeout_sec_ <= 0.0) return;

    bool timed_out = false;
    {
        std::lock_guard<std::mutex> lock(limit_mutex_);
        if (have_power_board_status_) {
            timed_out = (this->now() - last_power_board_status_time_).seconds() >
                        limit_status_timeout_sec_;
        }
    }

    if (timed_out) {
        {
            std::lock_guard<std::mutex> lock(limit_mutex_);
            limit_safety_fault_active_ = true;
        }
        setLimitStopState(true, "power board status timeout");
    }
}

void LiftController::setPositionCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    int pos_mm = static_cast<int>(msg->data);
    if (pos_mm < config_.min_position_mm || pos_mm > config_.max_position_mm) {
        RCLCPP_WARN(this->get_logger(), "Position %d mm out of range [%d, %d], ignored",
                    pos_mm, config_.min_position_mm, config_.max_position_mm);
        return;
    }
    if (rejectPositionIfBlockedByLimit("target_position", pos_mm)) return;
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
    if (rejectPositionIfBlockedByLimit("target_position_with_speed", pos_mm)) return;
    if (speed_rpm > config_.max_speed_rpm || speed_rpm < config_.min_speed_rpm) {
        RCLCPP_WARN(this->get_logger(), "Speed %d rpm out of range [%d, %d], clamped",
                    speed_rpm, config_.min_speed_rpm, config_.max_speed_rpm);
    }
    RCLCPP_INFO(this->get_logger(), "Moving to position: %d mm at %d rpm", pos_mm, speed_rpm);
    motor_driver_->setPosition(pos_mm, speed_rpm);
}

void LiftController::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        if (rejectCommandIfSafetyFault("enable_cmd")) return;
        motor_driver_->enable();
    } else {
        motor_driver_->disable();
    }
}

void LiftController::setZeroCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        if (rejectCommandIfSafetyFault("zero_cmd")) return;
        motor_driver_->setZero();
    }
}

void LiftController::clearErrorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        motor_driver_->clearError();
    }
}

} // namespace lift_controller
