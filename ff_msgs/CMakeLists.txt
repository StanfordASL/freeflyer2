cmake_minimum_required(VERSION 3.8)
project(ff_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose2D.msg"
  "msg/Pose2DStamped.msg"
  "msg/Twist2D.msg"
  "msg/Twist2DStamped.msg"
  "msg/Wrench2D.msg"
  "msg/Wrench2DStamped.msg"
  "msg/FreeFlyerState.msg"
  "msg/FreeFlyerStateStamped.msg"
  "msg/WheelVelCommand.msg"
  "msg/ThrusterCommand.msg"
  "msg/ThrusterPWMCommand.msg"
  "msg/ControllerMetrics.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
