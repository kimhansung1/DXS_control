cmake_minimum_required(VERSION 3.5)
project(control_package)

# C++ 표준 설정 (C++14 or C++17 등 필요에 따라 변경)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 노드 소스 파일
add_executable(control_node src/control_node.cpp)

# 타겟 링크 라이브러리
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# 타겟 인클루드 디렉토리 설정
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# install 타겟 설정
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include/
)

# ament 패키지 설정
ament_package()
