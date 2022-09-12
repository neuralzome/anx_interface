#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>

#include <ros/ros.h>
#include <hermes_interface_msgs/PhoneState.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "hermes_interface/asset_manager_interface.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class PhoneManager{
public:
  struct Phone{
    std::string id = "0";
    bool streaming = false;

    int port;
    std::unique_ptr<zmq::context_t> ctx_ptr;
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<std::thread> thread_ptr;

    int seq = 0;
    ros::NodeHandle nh;
    ros::Publisher publisher;
  };

  PhoneManager(AssetManagerInterface* asset_manager);
  void Start();
  void OnStateChange(nlohmann::json state);
private:
  void PhoneThread();
  bool IsPresent(nlohmann::json& state);

  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;

  Phone phone_;
};
