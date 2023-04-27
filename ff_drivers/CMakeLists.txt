cmake_minimum_required(VERSION 3.8)
project(ff_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_msgs REQUIRED)

add_library(pwm src/pwm.cpp)
target_include_directories(pwm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pwm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pwm rclcpp)

add_executable(thruster_node src/thruster_node.cpp)
target_link_libraries(thruster_node pwm)
ament_target_dependencies(thruster_node ff_msgs)

add_executable(test_single src/test_single.cpp)
target_link_libraries(test_single pwm)

# install nodes
install(TARGETS thruster_node test_single
        DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

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
