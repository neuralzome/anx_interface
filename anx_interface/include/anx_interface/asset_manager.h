#pragma once

#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <exception>
#include <chrono>

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <anx_interface_msgs/SendSignal.h>
#include <anx_interface_msgs/Signal.h>

#include "anx_interface/asset_interface.h"
#include "anx_interface/asset_manager_interface.h"
#include "anx_interface/imu_manager.h"
#include "anx_interface/gnss_manager.h"
#include "anx_interface/usb_serial_manager.h"
#include "anx_interface/camera_manager.h"
#include "anx_interface/phone_manager.h"
#include "anx_interface/speaker_manager.h"

class AssetManager : public AssetManagerInterface{
public:
  AssetManager();
  int GetFreePort() override;
  void ReturnFreePort(int port) override;
  void Start() override;
  void Stop() override;
  bool StartAsset(nlohmann::json msg) override;
  bool StopAsset(nlohmann::json msg) override;
  bool StartNonCoreAssetsCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);
  bool SignalCb(
      anx_interface_msgs::SendSignal::Request  &req,
      anx_interface_msgs::SendSignal::Response &res
  );
private:
  std::string GetIdentity();
  void AssetStateThread();

  // Subscribe and Unsubscribe from asset state stream
  bool Subscribe(bool subscribe);

  void OnStateChangeToAssets(const nlohmann::json& msg_json, bool core);
  void StartStopAsset(bool start, bool core);

  // Last received asset state
  std::string asset_state_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_non_core_asset_server_;
  bool non_core_asset_started_;

  ros::ServiceServer signal_server_;

  zmq::context_t ctx_;

  zmq::socket_t sub_asset_state_socket_;
  zmq::pollitem_t sub_asset_state_poll_;

  zmq::socket_t asset_state_socket_;
  zmq::pollitem_t asset_state_poll_;
  std::unique_ptr<std::thread> asset_state_thread_;

  zmq::socket_t start_asset_socket_;
  zmq::pollitem_t start_asset_poll_;

  zmq::socket_t stop_asset_socket_;
  zmq::pollitem_t stop_asset_poll_;

  zmq::socket_t get_identity_socket_;
  zmq::pollitem_t get_identity_poll_;

  zmq::socket_t send_signal_socket_;
  zmq::pollitem_t send_signal_poll_;

  int subscribe_asset_port_,
    start_asset_port_,
    stop_asset_port_,
    asset_state_port_,
    get_identity_port_,
    send_signal_port_;

  std::string anx_ip_;
  std::string linux_ip_;

  bool subscribed_;
  std::vector<int> port_pool_;

  std::vector<std::unique_ptr<AssetInterface>> assets_;
};
