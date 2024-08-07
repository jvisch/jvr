cmake_minimum_required(VERSION 3.8)
project(jvr_basic)

# C standard set to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# C++ standards set to 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(jvr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# executable TalkerNode
set(talker_node_SOURCES
  src/TalkerMain.cpp
  src/TalkerNode.cpp
)

add_executable(talker_node ${talker_node_SOURCES})

target_include_directories(talker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  talker_node
  jvr_interfaces
  rclcpp
)

install(TARGETS talker_node
  DESTINATION lib/${PROJECT_NAME}
)

# executable ListenerNode
set(listener_node_SOURCES
  src/ListenerMain.cpp
  src/ListenerNode.cpp
)

add_executable(listener_node ${listener_node_SOURCES})

target_include_directories(listener_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  listener_node
  jvr_interfaces
  rclcpp
)

install(TARGETS listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# testing
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
