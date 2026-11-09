cmake_minimum_required(VERSION 3.8)
project(bt_servers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_pkg REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ar_interfaces_2026 REQUIRED)

add_executable(rotate_server
  src/servers/rotate_server.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(move_server
  src/servers/move_server.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(sick_server_node
  src/servers/sick_pid_server.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(odom_server_node
  src/servers/odom_pid_server.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(align_arena_node
  src/servers/align_arena_server.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(brake_server
  src/servers/brake_server.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(sick_tuning
  src/TestingCodes/sick_tuning.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

# add_executable(align_server_node          # uncomment when source file exists
#   src/servers/align_server.cpp
#   include/robotlibpc/cpp/controller/pid.cpp
# )

target_include_directories(rotate_server PUBLIC include)
target_include_directories(move_server   PUBLIC include)
target_include_directories(brake_server  PUBLIC include)
target_include_directories(sick_server_node PUBLIC include)
target_include_directories(odom_server_node PUBLIC include)
target_include_directories(sick_tuning PUBLIC include)
target_include_directories(align_arena_node PUBLIC include)

ament_target_dependencies(rotate_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  action_pkg
  std_srvs
)

ament_target_dependencies(move_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  action_pkg
  std_srvs
)

ament_target_dependencies(sick_server_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  ar_interfaces_2026
)

ament_target_dependencies(odom_server_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  ar_interfaces_2026
)

ament_target_dependencies(brake_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  action_pkg
  std_srvs
)

ament_target_dependencies(sick_tuning
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  ar_interfaces_2026
)



ament_target_dependencies(align_arena_node   
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  ar_interfaces_2026
)

install(TARGETS
  rotate_server
  move_server
  brake_server
  sick_server_node
  odom_server_node
  sick_tuning
  align_arena_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()