cmake_minimum_required(VERSION 3.8)
project(ff_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_msgs REQUIRED)
find_package(ff_params REQUIRED)

add_library(ctrl_lib
    src/ll_ctrl.cpp
    src/pwm_ctrl.cpp
    src/wrench_ctrl.cpp
    src/keyboard_ctrl.cpp
    src/linear_ctrl.cpp
)
target_include_directories(ctrl_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ctrl_lib PUBLIC cxx_std_17)
ament_target_dependencies(ctrl_lib rclcpp ff_msgs ff_params Eigen3)

ament_export_targets(ctrl_libTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ff_msgs ff_params Eigen3 eigen3_cmake_module)

# install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# install CPP libraries
install(
  TARGETS ctrl_lib
  EXPORT ctrl_libTarget
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(pwm_ctrl_cpp_node src/nodes/pwm_ctrl_node.cpp)
target_link_libraries(pwm_ctrl_cpp_node ctrl_lib)

add_executable(wrench_ctrl_cpp_node src/nodes/wrench_ctrl_node.cpp)
target_link_libraries(wrench_ctrl_cpp_node ctrl_lib)

add_executable(pd_ctrl_cpp_node src/nodes/pd_ctrl_node.cpp)
target_link_libraries(pd_ctrl_cpp_node ctrl_lib)
ament_target_dependencies(pd_ctrl_cpp_node geometry_msgs)

add_executable(key_teleop_cpp_node src/nodes/key_teleop_node.cpp)
target_link_libraries(key_teleop_cpp_node ctrl_lib)

# install CPP nodes
install(TARGETS pwm_ctrl_cpp_node wrench_ctrl_cpp_node pd_ctrl_cpp_node key_teleop_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# install Python libraries
ament_python_install_package(${PROJECT_NAME})

# install Python nodes
install(PROGRAMS
  scripts/opt_ctrl_py_node
  scripts/pd_ctrl_py_node
  scripts/safety_filter
  scripts/scp_pd_ctrl_py_node
  scripts/tto_scp_pd_ctrl_py_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # disable Python style checkers
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
