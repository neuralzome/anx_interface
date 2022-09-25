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
#include "ros/node_handle.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class UsbSerialManager{
public:
  struct UsbSerial{
    std::string name;

    struct {
      std::string id;
      int baud;
      std::string delimiter;
    } select;

    bool streaming = false;
    int pub_port;
    int sub_port;
    std::unique_ptr<zmq::socket_t> sub_socket_ptr;
    std::unique_ptr<zmq::socket_t> pub_socket_ptr;
    std::unique_ptr<zmq::pollitem_t> pub_poll_ptr;
    std::unique_ptr<std::thread> pub_thread_ptr;

    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
  };

  UsbSerialManager(AssetManagerInterface* asset_manager);
  void Start();
  void Stop();
  void OnStateChange(nlohmann::json state);
  void ToUsbSerialCb(const std_msgs::String::ConstPtr& usb_serial_ros_msg_ptr, UsbSerial* usb_serial);
private:
  zmq::context_t ctx_;
  void FromUsbSerialThread(UsbSerial* usb_serial);
  bool IsPresent(UsbSerial& usb_serial, nlohmann::json& state);

  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;
  bool started_;

  std::vector<UsbSerial> usb_serial_;
};
