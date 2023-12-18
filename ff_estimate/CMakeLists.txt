cmake_minimum_required(VERSION 3.8)
project(ff_estimate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_msgs REQUIRED)

# Estimation library
add_library(est_lib src/base_mocap_estimator.cpp)
target_include_directories(est_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(est_lib PUBLIC cxx_std_17)
ament_target_dependencies(est_lib rclcpp ff_msgs geometry_msgs)

ament_export_targets(est_libTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ff_msgs)

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
# add_executable(moving_avg_estimator_node src/moving_avg_estimator_node.cpp)
# target_link_libraries(moving_avg_estimator_node est_lib)

# install(TARGETS moving_avg_estimator_node
#         DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

# kalman filter estimator node
add_executable(kalman_filter_node src/kalman_filter_node.cpp)
target_link_libraries(kalman_filter_node est_lib)

install(TARGETS kalman_filter_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
