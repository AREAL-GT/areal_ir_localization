cmake_minimum_required(VERSION 3.14)
project(areal_ir_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ortools CONFIG REQUIRED)
find_package(aml_uros_gimbal_interfaces REQUIRED)

INCLUDE_DIRECTORIES(
  include
  launch
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

add_executable(ir_localization src/ir_localization.cpp)
ament_target_dependencies(ir_localization aml_uros_gimbal_interfaces 
  rclcpp std_msgs sensor_msgs geometry_msgs OpenCV ortools) 

target_link_libraries(ir_localization ortools::ortools)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  ir_localization
  DESTINATION lib/${PROJECT_NAME})

ament_package()
