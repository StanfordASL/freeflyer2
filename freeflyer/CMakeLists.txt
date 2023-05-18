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
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # disable Python style checkers
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(test/pd_ctrl_launch_test.py)
endif()

ament_package()
