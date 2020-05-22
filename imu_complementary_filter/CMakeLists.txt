cmake_minimum_required(VERSION 3.5)
project(meka_imu_complementary_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(imu_transformer REQUIRED)


include_directories(
  include
)

set(library_name complementary_filter)
set(executable_name ${PROJECT_NAME}_node)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  message_filters 
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  imu_transformer
)

## Declare a cpp library
add_library(${library_name} SHARED
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

#  include/imu_complementary_filter/complementary_filter.h
#  include/imu_complementary_filter/complementary_filter_ros.h
#)


# create complementary_filter_node executable
add_executable(${executable_name}
  src/complementary_filter_node.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)


install(TARGETS ${library_name} ${executable_name}
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
