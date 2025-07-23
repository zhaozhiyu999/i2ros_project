cmake_minimum_required(VERSION 3.0.2)
project(planning_package)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msg_interfaces
)
find_package(nlohmann_json 3.2.0 REQUIRED)
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs
    visualization_msgs
    tf2 
    tf2_ros 
    tf2_geometry_msgs
    msg_interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

# 智能路径规划节点
add_executable(planning_node src/planning_node.cpp)
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_node ${catkin_LIBRARIES} nlohmann_json::nlohmann_json)

# 轨迹可视化节点
add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
add_dependencies(trajectory_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_visualizer ${catkin_LIBRARIES})

# Mock轨迹发布器（仅在文件存在时编译）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/mock_trajectory_publisher.cpp")
    add_executable(mock_trajectory_publisher src/mock_trajectory_publisher.cpp)
    add_dependencies(mock_trajectory_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(mock_trajectory_publisher ${catkin_LIBRARIES})
    set(MOCK_TARGET mock_trajectory_publisher)
else()
    set(MOCK_TARGET "")
endif()

# 安装规则
install(TARGETS planning_node trajectory_visualizer ${MOCK_TARGET}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)