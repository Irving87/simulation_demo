cmake_minimum_required(VERSION 2.8.3)
project(service_demo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)





## Generate services in the 'srv' folder
  add_service_files(
    FILES
    AddTwoInts.srv
  )



## Generate added messages and services with any dependencies listed here
 generate_messages(
    DEPENDENCIES
    std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES service_demo
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
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

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server service_demo_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client service_demo_gencpp)
