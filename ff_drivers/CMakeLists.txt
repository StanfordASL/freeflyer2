cmake_minimum_required(VERSION 3.8)
project(ff_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_msgs REQUIRED)
find_package(ff_control REQUIRED)

add_library(driver_lib src/pwm.cpp src/gpio.cpp)
target_include_directories(driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(driver_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(driver_lib rclcpp)

add_executable(thruster_node src/thruster_node.cpp)
target_link_libraries(thruster_node driver_lib)
ament_target_dependencies(thruster_node ff_msgs)

# tests
# TODO(alvin): remove pwm stuff
add_executable(test_single src/tests/test_single.cpp)
target_link_libraries(test_single driver_lib)

add_executable(test_all_thrusters src/tests/test_all_thrusters.cpp)
ament_target_dependencies(test_all_thrusters rclcpp ff_msgs ff_control)

# install nodes
install(TARGETS thruster_node test_single test_all_thrusters
        DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # disable Python style checkers
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
