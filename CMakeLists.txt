cmake_minimum_required(VERSION 2.8.3)
project(ros_essentials_cpp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  cv_bridge
  image_transport
  message_generation
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   IoTSensor.msg 
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES 
   AddTwoInts.srv
   )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_essentials_cpp
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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
  ${OpenCV_INCLUDE_DIRS}
)


add_library(utility_lib src/topic04_perception02_laser/laserscan/utility_lib.cpp)

#LaserScan
add_library(laserscan_lib src/topic04_perception02_laser/laserscan/LaserScanner.cpp)
target_link_libraries(laserscan_lib ${catkin_LIBRARIES})
add_dependencies(laserscan_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(laserscan_lib utility_lib)

#talker
add_executable(talker_node src/topic01_basics/talker_listener/talker.cpp)
target_link_libraries (talker_node ${catkin_LIBRARIES})
#add_dependencies(talker gaitech_doc_generate_message_cpp)

#listener
add_executable(listener_node src/topic01_basics/talker_listener/listener.cpp)
target_link_libraries (listener_node ${catkin_LIBRARIES})


#robot cleaner
add_executable(turtlesim_clean_node src/topic02_motion/turtlesim/robot_cleaner.cpp)
target_link_libraries (turtlesim_clean_node ${catkin_LIBRARIES})


add_executable(add_two_ints_server src/topic01_basics/service/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server ros_essentials_cpp_gencpp)

add_executable(add_two_ints_client src/topic01_basics/service/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client ros_essentials_cpp_gencpp)

add_executable(scan_subscriber_cpp src/topic04_perception02_laser/scan_subscriber.cpp)
target_link_libraries(scan_subscriber_cpp ${catkin_LIBRARIES})
target_link_libraries(scan_subscriber_cpp laserscan_lib)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(read_video_cpp src/topic03_perception/cpp/read_video.cpp)
target_link_libraries(read_video_cpp ${catkin_LIBRARIES})
target_link_libraries(read_video_cpp ${OpenCV_LIBRARIES})

add_executable(open_copy_cpp src/topic03_perception/cpp/open_copy.cpp)
target_link_libraries(open_copy_cpp ${catkin_LIBRARIES})
target_link_libraries(open_copy_cpp ${OpenCV_LIBRARIES})

add_executable(image_pub_sub_cpp src/topic03_perception/cpp/image_pub_sub.cpp)
target_link_libraries(image_pub_sub_cpp ${catkin_LIBRARIES})
target_link_libraries(image_pub_sub_cpp ${OpenCV_LIBRARIES})