cmake_minimum_required(VERSION 3.5)
project(gb_perception_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  image_geometry
  image_transport
  sensor_msgs
  tf2
  tf2_ros
  cv_bridge
  tf2_geometry_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/gb_perception_utils/Perceptor3D.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(perceptor_main
  src/perceptor_main.cpp
)
ament_target_dependencies(perceptor_main ${dependencies})
target_link_libraries(perceptor_main ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  perceptor_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

endif()


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
