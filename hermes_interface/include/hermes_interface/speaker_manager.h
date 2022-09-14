#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>
#include <exception>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "hermes_interface/asset_manager_interface.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class SpeakerManager{
public:
  struct Speaker{
    std::string name;

    struct {
      std::string id;
      std::string language;
    } select;

    bool streaming = false;
    int sub_port;
    std::unique_ptr<zmq::context_t> sub_ctx_ptr;
    std::unique_ptr<zmq::socket_t> sub_socket_ptr;

    ros::NodeHandle nh;
    ros::Subscriber subscriber;
  };

  SpeakerManager(AssetManagerInterface* asset_manager);
  void Start();
  void Stop();
  void OnStateChange(nlohmann::json state);
  void ToSpeakerCb(const std_msgs::String::ConstPtr& speaker_msg_ptr, Speaker* speaker);
private:
  bool IsPresent(Speaker& speaker, nlohmann::json& state);

  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;
  bool started_;

  std::vector<Speaker> speaker_;
};
