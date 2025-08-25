cmake_minimum_required(VERSION 3.8)
project(ff_estimate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_msgs REQUIRED)

# Estimation library
add_library(est_lib src/base_mocap_estimator.cpp)
target_include_directories(est_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(est_lib PUBLIC cxx_std_17)
ament_target_dependencies(est_lib rclcpp ff_msgs geometry_msgs Eigen3)

ament_export_targets(est_libTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ff_msgs Eigen3 eigen3_cmake_module)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS est_lib
  EXPORT est_libTarget
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# motion capture estimator node
add_executable(moving_avg_estimator_node src/moving_avg_estimator_node.cpp)
target_link_libraries(moving_avg_estimator_node est_lib)

#add Kalman filter node
add_executable(ConstVelKF src/ConstVelKF.cpp)
target_link_libraries(ConstVelKF est_lib)

install(TARGETS moving_avg_estimator_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
