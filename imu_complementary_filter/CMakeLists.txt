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
find_package(imu_transformer REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(library_name ${PROJECT_NAME})
## Declare a cpp library
include_directories(include)

set(library_name ${PROJECT_NAME})

set(executable_name complementary_filter_node)

add_executable(${executable_name}
  src/complementary_filter_node.cpp
)

add_library(${library_name}
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  message_filters 
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  imu_transformer
  tf2_geometry_msgs
)


ament_target_dependencies(${library_name}
  ${dependencies}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries( ${executable_name} ${library_name}
)


install(TARGETS ${executable_name} ${library_name}
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
ament_package()
