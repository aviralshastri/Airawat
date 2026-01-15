cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetJointAngle.srv"
  "srv/SetArmState.srv"
  "srv/EmergencyStop.srv"
  "srv/StopAndGoToIdeal.srv"
  "srv/GetCurrentAngle.srv"
  DEPENDENCIES std_msgs
)

# Build smooth_trajectory library (static to avoid linking issues)
add_library(smooth_trajectory STATIC src/smooth_trajectory.cpp)
target_include_directories(smooth_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build arm_controller node
add_executable(arm_controller_node src/arm_controller.cpp)
target_include_directories(arm_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_controller_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(arm_controller_node smooth_trajectory)

# Link the generated service interfaces (new method - no deprecation warning)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(arm_controller_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  arm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()