cmake_minimum_required(VERSION 2.8.3)
project(hermes_interface_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    Signal.msg
    PhoneState.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    SendSignal.srv
)

generate_messages(
 DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
)
