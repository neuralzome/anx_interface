cmake_minimum_required(VERSION 2.8.3)
project(anx_interface_msgs)

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
    Battery.msg
    Cpu.msg
    CpuFreq.msg
    PhoneState.msg
    Ram.msg
    Signal.msg
    Storage.msg
    Thermal.msg
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
