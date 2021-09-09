cmake_minimum_required(VERSION 3.8)
project(uav_inspections_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

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

add_executable(simple_goal_publisher src/simple_goal_publisher.cpp)
ament_target_dependencies(simple_goal_publisher
    rclcpp
    px4_msgs
    moveit_msgs
    geometry_msgs
)
install(TARGETS simple_goal_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(uav_moveit src/moveit_planning.cpp)
ament_target_dependencies(uav_moveit
    rclcpp
    tf2_ros
    nav_msgs
    moveit_msgs
    moveit_common
    geometry_msgs
    trajectory_msgs
    tf2_geometry_msgs
    moveit_ros_planning_interface
)
install(TARGETS uav_moveit
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes 
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
