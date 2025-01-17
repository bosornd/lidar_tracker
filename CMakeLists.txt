cmake_minimum_required(VERSION 3.8)
project(lidar_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(tracker_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lidar_tracker src/l_shape_tracker.cpp src/main.cpp src/kalman-cpp/kalman.cpp src/datmo.cpp src/cluster.cpp)
ament_target_dependencies(lidar_tracker rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs tracker_msgs Eigen3)

install(DIRECTORY
  launch config
	DESTINATION share/${PROJECT_NAME})

install(TARGETS
  lidar_tracker
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