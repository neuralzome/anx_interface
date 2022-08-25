#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "hermes_interface/asset_manager_interface.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

/*
   * Tasks: Subscribe to array of imu streams and publishes corresponding imu ros msgs.
*/
class ImuManager{
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
    std::unique_ptr<zmq::context_t> ctx_ptr;
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<std::thread> thread_ptr;
  };

  ImuManager(AssetManagerInterface* asset_manager);
  void Start();
  void OnStateChange(nlohmann::json state);
private:
  void ImuThread(Imu* imu);
  bool IsPresent(Imu& imu, nlohmann::json& state);
  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;

  std::vector<Imu> imu_;
};
