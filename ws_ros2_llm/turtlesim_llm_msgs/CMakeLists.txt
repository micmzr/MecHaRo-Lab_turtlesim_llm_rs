cmake_minimum_required(VERSION 3.12)

project(turtlesim_llm_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(action_files
  "action/FollowWaypoints.action"
  DEPENDENCIES geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()