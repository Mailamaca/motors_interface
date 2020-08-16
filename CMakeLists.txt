cmake_minimum_required(VERSION 3.5)
project(motors_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find library
find_library(WIRINGPI_LIBRARIES wiringPi)
find_library(CRYPT_LIBRARIES crypt)
find_library(RT_LIBRARIES rt)

if(!WIRINGPI_LIBRARIES)
  message([AUTHOR_WARNING] wiringPiNOTfound!)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(maila_msgs REQUIRED)

add_executable(motors_interface src/motors_interface.cpp
		src/pca9685_motors.cpp
		src/pwm_motor.cpp
    src/pca9685.c)
ament_target_dependencies(
		motors_interface
		rclcpp
		std_msgs
		maila_msgs)

#include_directories(${WIRINGPI_INCLUDE_DIRS}) 
target_link_libraries(${PROJECT_NAME}
 ${WIRINGPI_LIBRARIES}
 ${CRYPT_LIBRARIES}
 ${RT_LIBRARIES}
)


target_include_directories(motors_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS motors_interface
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
