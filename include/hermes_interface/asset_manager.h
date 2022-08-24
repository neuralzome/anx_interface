#pragma once

#include "ros/node_handle.h"
#include <memory>
#include <thread>
#include <string>

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

#include <ros/ros.h>

class AssetManager{
public:
  AssetManager();
  void Start();
  void Stop();
private:
  void AssetStateThread();
  bool Subscribe(bool subscribe);

  ros::NodeHandle nh_;

  zmq::context_t sub_asset_state_ctx_;
  zmq::socket_t sub_asset_state_socket_;

  zmq::context_t asset_state_ctx_;
  zmq::socket_t asset_state_socket_;
  std::unique_ptr<std::thread> asset_state_thread_;

  zmq::context_t start_asset_state_ctx_;
  zmq::socket_t start_asset_state_socket_;

  zmq::context_t stop_asset_state_ctx_;
  zmq::socket_t stop_asset_state_socket_;

  int subscribe_asset_port_,
               start_asset_port_,
               stop_asset_port_,
               asset_state_port_;
  std::string hermes_ip_;

  bool subscribed_;
};
