cmake_minimum_required(VERSION 3.0.0)
project(hermes_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(cppzmq)
find_package(nlohmann_json 3.2.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(
  hermes_interface
  src/hermes_interface_node.cpp
  src/hermes_interface.cpp
  src/asset_manager.cpp
  src/imu_manager.cpp

)

target_link_libraries(
  hermes_interface
  ${catkin_LIBRARIES}
  cppzmq
  nlohmann_json::nlohmann_json
)

option(BUILD_TESTS "Build tests" OFF) #OFF by default

if(BUILD_TESTS)
  add_executable(
    advanced_parameter_server
    test/advanced_parameter_server/advanced_parameter_server.cpp
  )

  target_link_libraries(
    advanced_parameter_server
    ${catkin_LIBRARIES}
  )

  add_executable(
    json_parsing
    test/json_parsing/json_parsing.cpp
  )

  target_link_libraries(
    json_parsing
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
  )
endif()
