#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "hermes_interface/asset_manager_interface.h"

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

class CameraManager{
public:
  struct Camera{
    std::string name;
    std::string frame_id;

    struct {
      std::string id;
      struct {
        int fps;
        int width;
        int height;
        std::string pixel_format;
      } stream;
      int compression_quality;
    } select;

    std::string camera_info_uri;

    bool streaming = false;
    int port;
    std::unique_ptr<zmq::context_t> ctx_ptr;
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<std::thread> thread_ptr;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_ptr;

    int seq = 0;
    image_transport::CameraPublisher publisher;
  };

  CameraManager(AssetManagerInterface* asset_manager);
  void Start();
  void OnStateChange(nlohmann::json state);
private:
  void CameraThread(Camera* camera);
  bool IsPresent(Camera& camera, nlohmann::json& state);

  ros::NodeHandle nh_;
  AssetManagerInterface* asset_manager_;
  std::string hermes_ip_;

  std::vector<Camera> camera_;
};
