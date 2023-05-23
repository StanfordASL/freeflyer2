cmake_minimum_required(VERSION 3.8)
project(freeflyer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ff_control REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_params REQUIRED)
find_package(ff_viz REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # disable ROS2 Python style checkers
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()

  # check Python style with black
  ament_add_pytest_test(black_formatting test/black_formatting.py)

  # test both implementations of PD controller
  add_launch_test(test/pd_ctrl_launch_test.py TARGET pd_ctrl_cpp_test ARGS "impl:=cpp")
  add_launch_test(test/pd_ctrl_launch_test.py TARGET pd_ctrl_py_test ARGS "impl:=py")
endif()

ament_package()
