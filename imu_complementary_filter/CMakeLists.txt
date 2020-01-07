cmake_minimum_required(VERSION 3.5)
project(imu_complementary_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(library_name ${PROJECT_NAME})
set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  message_filters 
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
)

## Declare a cpp library
add_library(${library_name}
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
)

target_include_directories(${library_name}
  PUBLIC include
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

#  include/imu_complementary_filter/complementary_filter.h
#  include/imu_complementary_filter/complementary_filter_ros.h
#)


# create complementary_filter_node executable
add_executable(complementary_filter_node
  src/complementary_filter_node.cpp)

ament_target_dependencies(complementary_filter_node
  ${dependencies}
)

target_include_directories(complementary_filter_node
  PUBLIC include
)

target_link_libraries(complementary_filter_node ${library_name})

install(TARGETS imu_complementary_filter complementary_filter_node
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION include 
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME} 
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
