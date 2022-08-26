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
    std::unique_ptr<zmq::context_t> sub_ctx_ptr;
    std::unique_ptr<zmq::socket_t> sub_socket_ptr;
    std::unique_ptr<zmq::context_t> pub_ctx_ptr;
    std::unique_ptr<zmq::socket_t> pub_socket_ptr;
    std::unique_ptr<std::thread> sub_thread_ptr;
  };

  UsbSerialManager(AssetManagerInterface* asset_manager);
  void Start();
  void OnStateChange(nlohmann::json state);
private:
  void InUsbSerialThread(UsbSerial* usb_serial);
  bool IsPresent(UsbSerial& usb_serial, nlohmann::json& state);
  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;

  std::vector<UsbSerial> usb_serial_;
};
