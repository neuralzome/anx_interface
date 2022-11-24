#pragma once

#include <string>
#include <array>
#include <vector>
#include <memory>
#include <thread>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "anx_interface/asset_manager_interface.h"
#include "anx_interface/base64.h"

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
    std::unique_ptr<zmq::socket_t> socket_ptr;
    std::unique_ptr<zmq::pollitem_t> poll_ptr;
    std::unique_ptr<std::thread> thread_ptr;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_ptr;

    int seq = 0;
    std::unique_ptr<ros::NodeHandle> nh_ptr;
    image_transport::CameraPublisher publisher;
  };

  CameraManager(AssetManagerInterface* asset_manager);
  void Start();
  void Stop();
  void OnStateChange(nlohmann::json state);
private:
  zmq::context_t ctx_;
  void CameraThread(Camera* camera);
  bool IsPresent(Camera& camera, nlohmann::json& state);
  void PublishCameraStream(zmq::message_t& base64_encoder_jpeg_img, Camera* camera);

  AssetManagerInterface* asset_manager_;
  std::string anx_ip_;
  bool started_;

  std::vector<Camera> camera_;
};
