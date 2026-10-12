cmake_minimum_required(VERSION 3.8)
project(lift_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(lift_controller_lib
  src/can_interface.cpp
  src/motor_driver.cpp
  src/lift_controller.cpp
)

target_include_directories(lift_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(lift_controller_lib PRIVATE -Wno-unused-parameter)
target_link_libraries(lift_controller_lib Threads::Threads)

ament_target_dependencies(lift_controller_lib
  rclcpp std_msgs
)

add_executable(lift_controller_node src/main.cpp)
target_link_libraries(lift_controller_node lift_controller_lib Threads::Threads)
ament_target_dependencies(lift_controller_node rclcpp std_msgs)

install(TARGETS lift_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lift_controller_lib
  DESTINATION lib
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/setup_can.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_export_dependencies(rclcpp std_msgs)
ament_export_include_directories(include)

ament_package()
