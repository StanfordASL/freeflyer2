cmake_minimum_required(VERSION 3.8)
project(ff_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(robot_params_node src/robot_params_node.cpp)
target_compile_features(robot_params_node PUBLIC cxx_std_17)
ament_target_dependencies(robot_params_node rclcpp)

install(TARGETS robot_params_node
  DESTINATION lib/${PROJECT_NAME})

add_library(robot_params src/robot_params.cpp)
target_include_directories(robot_params PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_params PUBLIC cxx_std_17)
ament_target_dependencies(robot_params rclcpp)

ament_export_targets(robot_paramsTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_params
  EXPORT robot_paramsTarget
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
