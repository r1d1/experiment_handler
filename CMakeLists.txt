cmake_minimum_required(VERSION 2.8.3)
project(experimentHandler)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs sensor_msgs laser_geometry message_generation message_runtime habelar_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(/usr/lib/)
include_directories(${catkin_INCLUDE_DIRS} /usr/include/jsoncpp)

## Declare a cpp executable
add_executable(navExpManager src/NavExpManager.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(experimentHandler_node experimentHandler_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(navExpManager ${catkin_LIBRARIES})

