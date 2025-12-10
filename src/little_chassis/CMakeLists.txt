cmake_minimum_required(VERSION 3.8)
project(little_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${Boost_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Motion.msg"
  "msg/IoState.msg"
  "msg/Light.msg"
  "msg/Battery.msg"
  "msg/ChassisState.msg"
  "srv/IoControl.srv"
  "srv/LightControl.srv"
  "srv/MotionControl.srv"
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

add_library(little_chassis_node
  src/little_chassis_node.cpp
  src/topics/cmd_vel_subscriber.cpp
  src/topics/odom_publisher.cpp
  src/topics/chassis_state_publisher.cpp
  src/services/io_control_service.cpp
  src/services/light_control_service.cpp
  src/services/motion_control_service.cpp
  src/params_transfer/parameter_transfer.cpp)
ament_target_dependencies(little_chassis_node rclcpp geometry_msgs nav_msgs std_msgs tf2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(little_chassis_node Boost::system ${cpp_typesupport_target})

add_executable(little_chassis_exe src/main.cpp)
set_target_properties(little_chassis_exe PROPERTIES OUTPUT_NAME little_chassis)
ament_target_dependencies(little_chassis_exe rclcpp)

target_link_libraries(little_chassis_exe little_chassis_node Boost::system)

install(TARGETS
  little_chassis_node
  little_chassis_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME})

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
