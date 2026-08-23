cmake_minimum_required(VERSION 3.8)
project(argus_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(argus_core REQUIRED)

add_executable(dataset_relay_node
  src/dataset_relay_node.cpp
  src/replay_server.cpp
)

target_include_directories(dataset_relay_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${argus_core_INCLUDE_DIRS}
)

ament_target_dependencies(dataset_relay_node rclcpp argus_core)
ament_target_dependencies(dataset_relay_node rclcpp std_msgs argus_core)

install(TARGETS dataset_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
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
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
endif()


ament_package()
