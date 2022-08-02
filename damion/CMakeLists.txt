cmake_minimum_required(VERSION 3.0.2)
project(damion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy	
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  ublox_msgs
  ublox_gps
  message_generation
  darknet_ros_msgs
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


add_message_files(FILES object_info.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES damion
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs rospy ublox_msgs ublox_gps message_runtime darknet_ros_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(crack src/crack.cpp)
target_link_libraries(crack ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(yolo_detection src/yolo_detection.cpp)

target_link_libraries(yolo_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} )

