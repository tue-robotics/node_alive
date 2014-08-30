cmake_minimum_required(VERSION 2.8.3)
project(node_alive)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  rospy
  message_generation
)

# Generate services in the 'srv' folder
add_service_files(
   FILES
   ListNodesAlive.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
)
