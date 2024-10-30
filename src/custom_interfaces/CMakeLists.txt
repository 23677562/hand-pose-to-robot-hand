cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

set(msg_files
  "msg/HandLandmarks.msg"
  "msg/PoseKeypoints.msg"
  "msg/PoseKeypoint.msg"
)

# Generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandLandmarks.msg"
  "msg/PoseKeypoints.msg"
  "msg/PoseKeypoint.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
