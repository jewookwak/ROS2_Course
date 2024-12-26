cmake_minimum_required(VERSION 3.8)
project(ros2_term_project)

# 컴파일러 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OCTOMAP 설정
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
add_definitions(-DOCTOMAP_NODEBUGOUT)

# DYNAMICEDT 설정
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
set(dynamicEDT3D_DIR "/opt/ros/humble/share/dynamicEDT3D")
set(DYNAMICEDT3D_LIBRARIES "/opt/ros/humble/lib/libdynamicedt3d.so")

# ROS2 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_state_msgs REQUIRED)  # 추가: robot_state_msgs 패키지

# Python 스크립트 설치를 위한 ament_cmake_python 설정
find_package(ament_cmake_python REQUIRED)

set(ROS2_PACKAGES
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
  octomap_ros
  octomap_msgs
  nav_msgs
  tf2_geometry_msgs
  sensor_msgs
  robot_state_msgs  )

# include 디렉토리 설정
include_directories(${PROJECT_SOURCE_DIR}/include)

# 빌드 테스트 옵션
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# cmd_publisher_node 실행 파일 생성
add_executable(cmd_publisher_node
  src/cmd_publisher_node.cpp
  src/cmd_publisher.cpp
  src/map.cpp)
ament_target_dependencies(cmd_publisher_node ${ROS2_PACKAGES})
target_link_libraries(cmd_publisher_node
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
)
install(TARGETS cmd_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# Python 스크립트 설치
install(DIRECTORY
  scripts
  launch
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(PROGRAMS
  scripts/ddpg_agent.py
  DESTINATION lib/${PROJECT_NAME}
)

# map_exploration_node 실행 파일 생성
add_executable(map_exploration_node
  src/map_exploration_node.cpp
  src/map_exploration_env.cpp)
ament_target_dependencies(map_exploration_node ${ROS2_PACKAGES})
target_link_libraries(map_exploration_node
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
)
install(TARGETS map_exploration_node
  DESTINATION lib/${PROJECT_NAME})

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 패키지 선언
ament_package()
