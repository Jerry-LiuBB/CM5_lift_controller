#include <rclcpp/rclcpp.hpp>
#include "lift_controller/lift_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lift_controller::LiftController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}