#pragma once

#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <exception>

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

#include <ros/ros.h>

#include "hermes_interface/asset_manager_interface.h"
#include "hermes_interface/imu_manager.h"

class AssetManager : public AssetManagerInterface{
public:
  AssetManager();
  int GetFreePort() override;
  void Start() override;
  void Stop() override;
  bool StartAsset(nlohmann::json msg) override;
  bool StopAsset(nlohmann::json msg) override;
private:
  void AssetStateThread();
  bool Subscribe(bool subscribe);

  ros::NodeHandle nh_;

  zmq::context_t sub_asset_state_ctx_;
  zmq::socket_t sub_asset_state_socket_;

  zmq::context_t asset_state_ctx_;
  zmq::socket_t asset_state_socket_;
  std::unique_ptr<std::thread> asset_state_thread_;

  zmq::context_t start_asset_ctx_;
  zmq::socket_t start_asset_socket_;

  zmq::context_t stop_asset_ctx_;
  zmq::socket_t stop_asset_socket_;

  int subscribe_asset_port_,
               start_asset_port_,
               stop_asset_port_,
               asset_state_port_;
  std::string hermes_ip_;

  bool subscribed_;
  std::vector<int> port_pool_;

  ImuManager imu_manager_;
};
