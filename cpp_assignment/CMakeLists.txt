cmake_minimum_required(VERSION 3.8)
project(cpp_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pothole_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(image src/image.cpp)
ament_target_dependencies(image rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport image_geometry tf2 tf2_ros tf2_geometry_msgs OpenCV)

add_executable(cv_find_pothole_xy src/cv_find_pothole_xy.cpp)
ament_target_dependencies(cv_find_pothole_xy rclcpp std_msgs pothole_interfaces sensor_msgs geometry_msgs cv_bridge image_transport image_geometry tf2 tf2_ros tf2_geometry_msgs OpenCV)

add_executable(find_pothole_poses src/find_pothole_poses.cpp)
ament_target_dependencies(find_pothole_poses rclcpp std_msgs pothole_interfaces sensor_msgs geometry_msgs cv_bridge image_transport image_geometry tf2 tf2_ros tf2_geometry_msgs OpenCV)

install(TARGETS
  image
  find_pothole_poses
  cv_find_pothole_xy
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
