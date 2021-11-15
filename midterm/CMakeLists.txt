cmake_minimum_required(VERSION 3.0.2)
project(midterm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  camera.msg
  lidar.msg
  thermovision.msg
  emergency.msg
)

add_action_files(
  FILES
  route.action
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  brake.srv
  disinfection.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES midterm
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation message_runtime roscpp rospy std_msgs
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
)

add_executable(Camera_Sensor src/Camera_Sensor.cpp)
add_dependencies(Camera_Sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Camera_Sensor
  ${catkin_LIBRARIES}
)

add_executable(Control_Center src/Control_Center.cpp)
add_dependencies(Control_Center ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Control_Center
  ${catkin_LIBRARIES}
)

add_executable(Robot_Control src/Robot_Control.cpp)
add_dependencies(Robot_Control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Robot_Control
  ${catkin_LIBRARIES}
)

add_executable(Lidar_Sensor src/Lidar_Sensor.cpp)
add_dependencies(Lidar_Sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Lidar_Sensor
  ${catkin_LIBRARIES}
)

add_executable(Thermovision_Camera src/Thermovision_Camera.cpp)
add_dependencies(Thermovision_Camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Thermovision_Camera
  ${catkin_LIBRARIES}
)

add_executable(Emergency_Center src/Emergency_Center.cpp)
add_dependencies(Emergency_Center ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Emergency_Center
  ${catkin_LIBRARIES}
)

add_executable(Disinfection src/Disinfection.cpp)
add_dependencies(Disinfection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(Disinfection
  ${catkin_LIBRARIES}
)


