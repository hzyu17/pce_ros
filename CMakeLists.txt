cmake_minimum_required(VERSION 3.10)
project(pce_ros)

# === C++ standard ===
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# === Dependencies ===
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# === Library ===
add_library(${PROJECT_NAME} SHARED
  src/pce_optimization_task.cpp
  src/pce_planner.cpp
  src/pce_planner_manager.cpp
  src/visualizer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  eigen_stl_containers
  tf2_ros
  tf2_eigen_kdl
  visualization_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# === Plugin export ===
pluginlib_export_plugin_description_file(moveit_core pce_planner_plugin.xml)

# === Install ===
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES pce_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# === IMPORTANT: Install config and launch directories ===
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === Package ===
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning
  tf2_eigen_kdl
  visualization_msgs
  std_msgs
)
ament_package()