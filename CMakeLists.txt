cmake_minimum_required(VERSION 3.8)
project(bt_nav2_ergocub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(YARP COMPONENTS os REQUIRED)

set(DEPENDENCIES
  behaviortree_cpp_v3
  rclcpp
  std_srvs
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav2_util
)

include_directories(include)

set(LIBRARY_NAME ${PROJECT_NAME}_core)

add_library(${LIBRARY_NAME} SHARED
  src/decorator_on_bool.cpp
)
ament_target_dependencies(${LIBRARY_NAME} ${DEPENDENCIES})
target_compile_definitions(${LIBRARY_NAME} PRIVATE BT_PLUGIN_EXPORT)

add_library(is_goal_reached SHARED src/is_goal_reached.cpp)
list(APPEND plugin_libs is_goal_reached)
ament_target_dependencies(is_goal_reached ${DEPENDENCIES} YARP)
target_compile_definitions(is_goal_reached PRIVATE BT_PLUGIN_EXPORT)

add_library(is_path_valid SHARED src/is_path_valid.cpp)
list(APPEND plugin_libs is_path_valid)
ament_target_dependencies(is_path_valid ${DEPENDENCIES})
target_compile_definitions(is_path_valid PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  ${LIBRARY_NAME}
  is_goal_reached
  is_path_valid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY bt_descriptions/ DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()