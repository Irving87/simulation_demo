cmake_minimum_required(VERSION 2.8.3)
project(topic_demo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


add_message_files(
  FILES
  gps.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_demo
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


   add_executable(talker src/talker.cpp)
   add_dependencies(talker topic_demo_generate_messages_cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})

   add_executable(listener src/listener.cpp)
   add_dependencies(listener topic_demo_generate_messages_cpp)
   target_link_libraries(listener ${catkin_LIBRARIES})

