cmake_minimum_required(VERSION 3.8)
project(tr_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif() 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetWithImg.msg"
  "msg/FromSTM32.msg"
  "msg/ToSTM32.msg"
  "msg/Panel.msg"
  "msg/Panels.msg"
  "msg/Robot.msg"
  "msg/Robots.msg"
  "msg/debug/RobotDebug.msg"
  "msg/debug/RobotsDebug.msg"
  "msg/debug/LightBarPair.msg"
  "msg/debug/LightBar.msg"
  "msg/debug/Pose.msg"
  "msg/Point.msg"
  "srv/ListenSerial.srv"
  "srv/WriteSerial.srv"

  "msg/simulation/SimTeleopInput.msg"
  "msg/simulation/SimGroundTruth.msg"
  "msg/simulation/RobotGroundTruth.msg"
  DEPENDENCIES vision_msgs sensor_msgs builtin_interfaces geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
