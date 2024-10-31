cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
  
# Add p1_publisher executable
add_executable(p1_publisher src/p1_publisher.cpp)
ament_target_dependencies(p1_publisher rclcpp std_msgs)
install(TARGETS p1_publisher
  DESTINATION lib/${PROJECT_NAME})

# Add p1_subscriber executable
add_executable(p1_subscriber src/p1_subscriber.cpp)
ament_target_dependencies(p1_subscriber rclcpp std_msgs)
install(TARGETS p1_subscriber
  DESTINATION lib/${PROJECT_NAME})

# Add p2_fast_agent executable
add_executable(fast_agent src/p2_fast_agent.cpp)
ament_target_dependencies(fast_agent rclcpp std_msgs)
install(TARGETS fast_agent
  DESTINATION lib/${PROJECT_NAME})

# Add p2_slow_agent executable
add_executable(slow_agent src/p2_slow_agent.cpp)
ament_target_dependencies(slow_agent rclcpp std_msgs)
install(TARGETS slow_agent
  DESTINATION lib/${PROJECT_NAME})

find_package(ros2_interfaces_tutorial REQUIRED)

add_executable(custom_msg_publisher src/custom_msg_publisher.cpp)
ament_target_dependencies(custom_msg_publisher
  rclcpp
  ros2_interfaces_tutorial
)
install(TARGETS
  custom_msg_publisher
  DESTINATION lib/${PROJECT_NAME})  

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)
install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)
install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME})

add_executable(led_client src/toggle_led_client.cpp)
ament_target_dependencies(led_client rclcpp std_srvs)
install(TARGETS
  led_client
  DESTINATION lib/${PROJECT_NAME})

add_executable(led_service src/toggle_led_service.cpp)
ament_target_dependencies(led_service rclcpp std_srvs)
install(TARGETS
  led_service
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(cpp_node src/cpp_node.cpp)
target_include_directories(cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS cpp_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtlebot_controller src/turtlebot_controller.cpp)
ament_target_dependencies(turtlebot_controller rclcpp geometry_msgs turtlesim)
install(TARGETS 
  turtlebot_controller
  DESTINATION lib/${PROJECT_NAME})

add_executable(square src/square.cpp)
ament_target_dependencies(square rclcpp geometry_msgs turtlesim)
install(TARGETS 
  square
  DESTINATION lib/${PROJECT_NAME})
add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)
install(TARGETS
  minimal_param_node
  DESTINATION lib/${PROJECT_NAME})  
  
add_executable(turtlebot_visualizer src/turtlebot_visualizer.cpp)
ament_target_dependencies(turtlebot_visualizer rclcpp geometry_msgs turtlesim visualization_msgs)
install(TARGETS 
  turtlebot_visualizer
  DESTINATION lib/${PROJECT_NAME})
  
  
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

install(DIRECTORY launch 
        DESTINATION share/${PROJECT_NAME})


ament_package()
