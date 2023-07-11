cmake_minimum_required(VERSION 3.8)
project(anx_interface_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    anx_interface_ros2/imu_driver.py
    anx_interface_ros2/gnss_driver.py
    anx_interface_ros2/camera_driver.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
