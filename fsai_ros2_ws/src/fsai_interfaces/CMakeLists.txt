cmake_minimum_required(VERSION 3.8)
project(fsai_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone3D.msg"
  "msg/Cone3DArray.msg"
  "msg/VehicleControl.msg"
  "msg/DriveCommand.msg"
  "msg/VcuStatus.msg"
  "msg/WheelSpeeds.msg"
  "msg/InterfaceState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
