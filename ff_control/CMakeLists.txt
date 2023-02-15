cmake_minimum_required(VERSION 3.8)
project(ff_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ff_params REQUIRED)

add_executable(force_torque_ctrl_node src/force_torque_ctrl_node.cpp)
target_include_directories(force_torque_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(force_torque_ctrl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(force_torque_ctrl_node rclcpp)

install(TARGETS force_torque_ctrl_node
  DESTINATION lib/${PROJECT_NAME})

add_library(ctrl_lib src/ll_ctrl.cpp src/wrench_ctrl.cpp)
target_include_directories(ctrl_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ctrl_lib rclcpp std_msgs geometry_msgs ff_params)

ament_export_targets(ctrl_libTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs ff_params)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ctrl_lib
  EXPORT ctrl_libTarget
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
