cmake_minimum_required(VERSION 3.0.2)
project(node_alive)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

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
  CATKIN_DEPENDS message_runtime std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
