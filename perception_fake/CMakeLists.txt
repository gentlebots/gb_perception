cmake_minimum_required(VERSION 3.0.2)
project(perception_fake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  vision_msgs
  tf2_geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp tf2 tf2_ros vision_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Perception.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/perception_fake_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
