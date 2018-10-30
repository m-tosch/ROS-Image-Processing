cmake_minimum_required(VERSION 2.8.3)
project(camera_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(OpenCV)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
  sensor_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package( CATKIN_DEPENDS
   cv_bridge
   image_transport
   sensor_msgs
   message_runtime
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# image publisher
add_executable(image_publisher src/image_publisher.cpp)
add_dependencies(image_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# image_converter
add_executable(image_converter src/image_converter.cpp)
add_dependencies(image_converter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# image subscriber
add_executable(image_subscriber src/image_subscriber.cpp)
add_dependencies(image_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
