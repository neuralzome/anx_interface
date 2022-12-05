#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>

#include <ros/ros.h>
#include <anx_interface_msgs/PhoneState.h>
#include <anx_interface_msgs/CpuFreq.h>
#include <anx_interface_msgs/Thermal.h>

#include <xmlrpcpp/XmlRpcValue.h>

#include "anx_interface/asset_manager_interface.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class PhoneManager{
public:
  struct Phone{
    std::string id = "0";
    bool streaming = false;

    int port;
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<zmq::pollitem_t> poll_ptr;
    std::unique_ptr<std::thread> thread_ptr;

    int seq = 0;
    ros::NodeHandle nh;
    ros::Publisher publisher;
  };

  PhoneManager(AssetManagerInterface* asset_manager);
  void Start();
  void OnStateChange(nlohmann::json state);
private:
  zmq::context_t ctx_;
  void PhoneThread();
  bool IsPresent(nlohmann::json& state);

  AssetManagerInterface* asset_manager_;
  std::string anx_ip_;

  Phone phone_;
};
