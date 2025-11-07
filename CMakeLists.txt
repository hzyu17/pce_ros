cmake_minimum_required(VERSION 3.10)
project(pce_ros)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Release build type with optimizations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -DNDEBUG")

# Enable OpenMP for parallel processing
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message(STATUS "OpenMP enabled for parallelization")
endif()


find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  roscpp
  geometric_shapes
  eigen_stl_containers
  visualization_msgs
  tf2_ros
  tf2_eigen
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  LIBRARIES pce_ros
  CATKIN_DEPENDS 
    moveit_core 
    moveit_ros_planning 
    pluginlib 
  DEPENDS 
    EIGEN3
    YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# PCE MoveIt library
add_library(pce_ros
  src/pce_optimization_task.cpp
  src/pce_planner.cpp
  src/pce_planner_manager.cpp
  src/visualizer.cpp
)

target_link_libraries(pce_ros
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

# Install
install(TARGETS pce_ros
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES pce_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

message(STATUS "==========================================")
message(STATUS "PCE ROS - Build Configuration")
message(STATUS "==========================================")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "CXX Release Flags: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "==========================================")