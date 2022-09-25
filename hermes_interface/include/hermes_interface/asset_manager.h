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
#include <hermes_interface_msgs/SendSignal.h>
#include <hermes_interface_msgs/Signal.h>

#include "hermes_interface/asset_manager_interface.h"
#include "hermes_interface/imu_manager.h"
#include "hermes_interface/usb_serial_manager.h"
#include "hermes_interface/camera_manager.h"
#include "hermes_interface/phone_manager.h"
#include "hermes_interface/speaker_manager.h"

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
      hermes_interface_msgs::SendSignal::Request  &req,
      hermes_interface_msgs::SendSignal::Response &res
  );
private:
  std::string GetIdentity();
  void AssetStateThread();
  bool Subscribe(bool subscribe);
  std::string asset_state_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_server_;
  ros::ServiceServer signal_server_;
  bool non_core_asset_started_;

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
               send_signal_port_;;
  std::string hermes_ip_;
  std::string linux_ip_;

  bool subscribed_;
  std::vector<int> port_pool_;

  ImuManager imu_manager_;
  UsbSerialManager usb_serial_manager_;
  CameraManager camera_manager_;
  PhoneManager phone_manager_;
  SpeakerManager speaker_manager_;
};
