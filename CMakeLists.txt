cmake_minimum_required(VERSION 3.8)
project(depth_plane_seg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(custom_ros2_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include/depth_plane_seg
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(depth_plane_seg_node src/depth_plane_seg_node.cpp)
ament_target_dependencies(depth_plane_seg_node rclcpp sensor_msgs pcl_conversions pcl_ros custom_ros2_msgs)
target_link_libraries(depth_plane_seg_node ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(depth_plane_seg_node_debug src/depth_plane_seg_node_debug.cpp)
ament_target_dependencies(depth_plane_seg_node_debug rclcpp sensor_msgs pcl_conversions pcl_ros custom_ros2_msgs)
target_link_libraries(depth_plane_seg_node_debug ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  depth_plane_seg_node
  depth_plane_seg_node_debug
  DESTINATION lib/${PROJECT_NAME}
)

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
