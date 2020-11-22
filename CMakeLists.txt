cmake_minimum_required(VERSION 3.5)
project(motors_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(i2c_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(maila_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# motor_component
add_library(motor_component SHARED
  src/motor_component.cpp
  src/pwm_motor.cpp)
target_compile_definitions(motor_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(motor_component
  "i2c_interfaces"
	"std_msgs"
	"maila_msgs"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(motor_component "motor::MotorComponent")
set(node_plugins "${node_plugins}motor::MotorComponent;$<TARGET_FILE:motor_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  motor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
