#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>

#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "hermes_interface/asset_manager_interface.h"
#include "ros/node_handle.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class GnssManager{
public:
  struct Gnss{
    std::string name;

    struct {
      std::string id;
      int fps;
    } select;

    bool streaming = false;
    int port;
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<zmq::pollitem_t> poll_ptr;
    std::unique_ptr<std::thread> thread_ptr;

    int seq = 0;
    ros::NodeHandle nh;
    ros::Publisher publisher;
  };

  GnssManager(AssetManagerInterface* asset_manager);
  void Start();
  void Stop();
  void OnStateChange(nlohmann::json state);
private:
  zmq::context_t ctx_;
  void GnssThread(Gnss* gnss);
  bool IsPresent(Gnss& gnss, nlohmann::json& state);

  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;
  bool started_;

  std::vector<Gnss> gnss_;
};
