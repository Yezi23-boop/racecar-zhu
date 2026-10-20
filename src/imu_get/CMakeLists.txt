cmake_minimum_required(VERSION 3.8)
project(imu_get)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
#find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(geometry_msgs)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(publisher_imu_node src/publisher_imu.cpp src/transform.hpp)
ament_target_dependencies(publisher_imu_node rclcpp std_msgs)
ament_target_dependencies(publisher_imu_node rclcpp serial)
ament_target_dependencies(publisher_imu_node rclcpp sensor_msgs)
ament_target_dependencies(publisher_imu_node rclcpp rosidl_default_generators)
ament_target_dependencies(publisher_imu_node rclcpp nav_msgs)
ament_target_dependencies(publisher_imu_node rclcpp tf2_ros)
ament_target_dependencies(publisher_imu_node rclcpp tf2_msgs)
ament_target_dependencies(publisher_imu_node rclcpp tf2_geometry_msgs)

install(TARGETS
  publisher_imu_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(sub_imu_node src/sub_imu.cpp)
ament_target_dependencies(sub_imu_node rclcpp std_msgs)
ament_target_dependencies(sub_imu_node rclcpp std_msgs)
ament_target_dependencies(sub_imu_node rclcpp serial)
ament_target_dependencies(sub_imu_node rclcpp sensor_msgs)
ament_target_dependencies(sub_imu_node rclcpp rosidl_default_generators)
ament_target_dependencies(sub_imu_node rclcpp nav_msgs)
ament_target_dependencies(sub_imu_node rclcpp tf2_ros)
ament_target_dependencies(sub_imu_node rclcpp tf2_msgs)
ament_target_dependencies(sub_imu_node rclcpp tf2_geometry_msgs)
install(TARGETS
  sub_imu_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
