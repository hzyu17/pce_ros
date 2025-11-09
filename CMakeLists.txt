cmake_minimum_required(VERSION 3.10)
project(pce_ros)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GPU acceleration option (declare early, use later)
option(USE_GPU "Enable GPU acceleration with CuRobo" OFF)

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

# Find catkin and dependencies
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

# Catkin package
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

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

###########
## Build ##
###########

# PCE MoveIt library (CREATE TARGET FIRST!)
add_library(pce_ros
  src/pce_optimization_task.cpp
  src/pce_planner.cpp
  src/pce_planner_manager.cpp
  src/visualizer.cpp
)

# Link basic dependencies to pce_ros
target_link_libraries(pce_ros
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

# === GPU Support (AFTER target creation) ===
if(USE_GPU)
  message(STATUS "GPU acceleration enabled")
  
  # Find Python from virtual environment
  set(VENV_PATH "${CMAKE_CURRENT_SOURCE_DIR}/venv")
  
  if(EXISTS "${VENV_PATH}/bin/python3")
    message(STATUS "Found virtual environment: ${VENV_PATH}")
    set(Python3_EXECUTABLE "${VENV_PATH}/bin/python3")
    
    # Get Python paths from venv
    execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('include'))"
      OUTPUT_VARIABLE Python3_INCLUDE_DIRS
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
    execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))"
      OUTPUT_VARIABLE Python3_LIBRARY_DIRS
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
    message(STATUS "Python3 from venv:")
    message(STATUS "  Executable: ${Python3_EXECUTABLE}")
    message(STATUS "  Include: ${Python3_INCLUDE_DIRS}")
    message(STATUS "  Library dir: ${Python3_LIBRARY_DIRS}")
  else()
    message(WARNING "Virtual environment not found at ${VENV_PATH}")
    message(WARNING "Please run: ./scripts/setup_gpu_env.sh")
  endif()
  
  # Find Python and pybind11
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  
  # Find pybind11 (try venv first, then system)
  find_package(pybind11 QUIET HINTS "${VENV_PATH}/share/cmake/pybind11")
  if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found in venv, trying system...")
    find_package(pybind11 REQUIRED)
  endif()
  
  if(pybind11_FOUND)
    message(STATUS "pybind11 found")
  else()
    message(FATAL_ERROR "pybind11 not found! Install with: sudo apt install python3-pybind11")
  endif()
  
  # GPU collision bridge library
  add_library(gpu_collision_bridge
    src/gpu_collision_bridge.cpp
  )
  
  target_include_directories(gpu_collision_bridge PUBLIC
    ${Python3_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  )
  
  target_link_libraries(gpu_collision_bridge
    ${Python3_LIBRARIES}
    pybind11::embed
  )
  
  target_compile_definitions(gpu_collision_bridge PUBLIC USE_GPU)
  
  # Link GPU bridge to main library (pce_ros already exists now!)
  target_link_libraries(pce_ros
    gpu_collision_bridge
  )
  
  target_compile_definitions(pce_ros PRIVATE USE_GPU)
  
  message(STATUS "GPU support configured")
  
else()
  message(STATUS "GPU acceleration disabled (use -DUSE_GPU=ON to enable)")
endif()

#############
## Install ##
#############

install(TARGETS pce_ros
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if(USE_GPU)
  install(TARGETS gpu_collision_bridge
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

install(FILES pce_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

# Build summary
message(STATUS "==========================================")
message(STATUS "PCE ROS - Build Configuration")
message(STATUS "==========================================")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "CXX Release Flags: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "GPU Support: ${USE_GPU}")
message(STATUS "==========================================")