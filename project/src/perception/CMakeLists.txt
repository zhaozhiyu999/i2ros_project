cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  image_geometry
)

find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(map_builder_node src/map_builder_node.cpp)
target_link_libraries(map_builder_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable(depth_to_cloud_node src/depth_to_cloud_node.cpp)
target_link_libraries(depth_to_cloud_node ${catkin_LIBRARIES})

add_executable(traffic_light_node src/traffic_light_node.cpp)
target_link_libraries(traffic_light_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(front_hazard_node src/front_hazard_node.cpp)
target_link_libraries(front_hazard_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
