cmake_minimum_required(VERSION 3.8)
project(jvr_sim)

# C standard set to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# C++ standards set to 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(robot_state_publisher REQUIRED)


### Gazebo plugins

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Garden
else()
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()

# FullSystem plugin (from example)
add_library(FullSystem
  SHARED
  src/FullSystem.cpp
)

target_include_directories(
  FullSystem PRIVATE include
)

target_link_libraries(FullSystem PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# SweepSensor plugin
add_library(SweepSensor
  SHARED
  src/SweepSensor.cpp
)

target_include_directories(
  SweepSensor PRIVATE include
)

target_link_libraries(SweepSensor PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})


# Install plugins to 'lib' dir
install(
  TARGETS FullSystem SweepSensor
  DESTINATION lib/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

#######

# Install launch files.
install(DIRECTORY
  description
  launch
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
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
endif()

ament_package()
