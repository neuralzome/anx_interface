cmake_minimum_required(VERSION 3.0.0)
project(anx_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  camera_info_manager
  image_transport
  cv_bridge
  anx_interface_msgs
  nmea_msgs
)

find_package(cppzmq)
find_package(nlohmann_json 3.2.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    camera_info_manager
    image_transport
    cv_bridge
    std_srvs
    anx_interface_msgs
    nmea_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(
  anx_interface
  src/anx_interface_node.cpp
  src/anx_interface.cpp
  src/asset_manager.cpp
  src/imu_manager.cpp
  src/gnss_manager.cpp
  src/usb_serial_manager.cpp
  src/camera_manager.cpp
  src/base64.cpp
  src/phone_manager.cpp
  src/speaker_manager.cpp
)

add_dependencies(anx_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  anx_interface
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

install(TARGETS
  anx_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param)
