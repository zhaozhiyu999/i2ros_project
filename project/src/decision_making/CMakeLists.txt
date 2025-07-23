cmake_minimum_required(VERSION 3.0.2)
project(decision_making)

# 若以后想用 C++17 语法可保留
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs            # 仅用到 Bool / String
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

################################################################################
#  Build decision_node
################################################################################
add_executable(decision_node src/decision_node.cpp)
target_link_libraries(decision_node ${catkin_LIBRARIES})
add_dependencies(decision_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
