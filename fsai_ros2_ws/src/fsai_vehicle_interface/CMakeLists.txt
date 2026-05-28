cmake_minimum_required(VERSION 3.14)
project(fsai_vehicle_interface)

# ── C++ standard ──────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fsai_interfaces REQUIRED)

# ── Vendored FS-AI API (C library) ────────────────────────────────────────────
#
# Build the two C source files from vendor/ as a static library so they are
# compiled with C semantics (no -Wpedantic C++ warnings, C99 VLAs, etc.).
# The node links against this library; the include path is added as PUBLIC so
# that any downstream target picking up this node as a dep also sees the header.
#
add_library(fs_ai_api_vendor STATIC
  vendor/can.c
  vendor/fs-ai_api.c
)
target_include_directories(fs_ai_api_vendor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor>
)
# The vendor C code uses POSIX pthreads and rt (clock_gettime on older kernels)
target_link_libraries(fs_ai_api_vendor pthread rt)

# Suppress pedantic warnings from vendored code — it is not ours to fix
target_compile_options(fs_ai_api_vendor PRIVATE
  -Wno-pedantic -Wno-unused-parameter
)

# ── Node executable ───────────────────────────────────────────────────────────
add_executable(vehicle_interface_node
  src/main.cpp
  src/vehicle_interface_node.cpp
  src/state_machine.cpp
)

target_include_directories(vehicle_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor>
)

target_link_libraries(vehicle_interface_node
  fs_ai_api_vendor
)

ament_target_dependencies(vehicle_interface_node
  rclcpp
  std_msgs
  sensor_msgs
  fsai_interfaces
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
