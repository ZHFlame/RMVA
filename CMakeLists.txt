cmake_minimum_required(VERSION 3.8)
project(team25_challenge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(pose_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(vision_node src/vision_node.cpp
        src/vision/armor.cpp
        src/vision/sphere.cpp
        src/vision/detector.cpp)
add_executable(shooter_node src/shooter_node.cpp
        src/vision/armor.cpp
        src/vision/sphere.cpp
        src/vision/detector.cpp
        src/shooter/calculate.cpp)
target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}  
)
target_include_directories(shooter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}  
)
target_compile_features(vision_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(shooter_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  vision_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
  gazebo_msgs
  sensor_msgs
)
ament_target_dependencies(
  shooter_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
  gazebo_msgs
  sensor_msgs
)
target_link_libraries(vision_node  PRIVATE ${typesupport_target})
target_link_libraries(shooter_node  PRIVATE ${typesupport_target})
install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS shooter_node
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

ament_package()
