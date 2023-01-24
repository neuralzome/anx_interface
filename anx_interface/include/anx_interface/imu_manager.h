#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "anx_interface/asset_interface.h"
#include "anx_interface/asset_manager_interface.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class ImuManager : public AssetInterface{
public:
  struct Imu{
    std::string name;

    struct {
      std::string id;
      int fps;
    } select;

    struct {
      std::array<double, 3> orientation_covariance_diagonal;
      std::array<double, 3> angular_velocity_covariance_diagonal;
      std::array<double, 3> linear_acceleration_covariance_diagonal;
    } covariance;

    bool streaming = false;
    int port;
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<zmq::pollitem_t> poll_ptr;
    std::unique_ptr<std::thread> thread_ptr;

    int seq = 0;
    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Publisher raw_publisher;
    ros::Publisher magnetic_field_publisher;
  };

  ImuManager(AssetManagerInterface* asset_manager);
  std::string Name() override;
  bool IsCore() override;
  void Start() override;
  void Stop() override;
  void OnStateChange(nlohmann::json state) override;
private:
  zmq::context_t ctx_;
  void ImuThread(Imu* imu);
  bool IsPresent(Imu& imu, nlohmann::json& state);

  AssetManagerInterface* asset_manager_;
  std::string anx_ip_;
  bool started_;

  std::vector<Imu> imu_;
};
