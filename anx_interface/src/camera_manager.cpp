#include "anx_interface/camera_manager.h"

CameraManager::CameraManager(AssetManagerInterface* asset_manager)
  : started_(false){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  if(!nh_private.getParam("anx_ip", this->anx_ip_)){
    this->anx_ip_ = "localhost";
  }
}

std::string CameraManager::Name(){
  return "camera";
}

bool CameraManager::IsCore(){
  return false;
}

void CameraManager::Start(){
  if(this->started_){
    return;
  }else{
    this->started_ = true;
  }
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("camera")){
    return;
  }
  // Populate camera_ from parameter server
  XmlRpc::XmlRpcValue camera_params;
  nh_private.getParam("camera", camera_params);
  for(int i=0; i<camera_params.size(); i++){
    this->camera_.emplace_back();
    Camera* camera = &this->camera_.back();

    camera->name = std::string(camera_params[i]["name"]);
    camera->nh_ptr = std::make_unique<ros::NodeHandle>(camera->name);
    camera->frame_id = std::string(camera_params[i]["frame_id"]);
    camera->select.id = std::string(camera_params[i]["select"]["id"]);
    camera->select.stream.fps = int(camera_params[i]["select"]["stream"]["fps"]);
    camera->select.stream.width = int(camera_params[i]["select"]["stream"]["width"]);
    camera->select.stream.height = int(camera_params[i]["select"]["stream"]["height"]);
    camera->select.stream.pixel_format = std::string(
        camera_params[i]["select"]["stream"]["pixel_format"]
    );
    camera->select.compression_quality = int(
        camera_params[i]["select"]["compression_quality"]
    );

    camera->camera_info_uri = std::string(camera_params[i]["camera_info_uri"]);
    camera->camera_info_manager_ptr = std::make_unique<camera_info_manager::CameraInfoManager>(
        *camera->nh_ptr,
        camera->name,
        camera->camera_info_uri
    );
		if (!camera->camera_info_manager_ptr->isCalibrated()){
      camera->camera_info_manager_ptr->setCameraName(camera->name);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = camera->frame_id;
      camera_info.width = camera->select.stream.width;
      camera_info.height = camera->select.stream.height;
      camera->camera_info_manager_ptr->setCameraInfo(camera_info);
    }


    camera->socket_ptr = std::make_unique<zmq::socket_t>(
        this->ctx_,
        zmq::socket_type::sub
    );
    camera->poll_ptr = std::make_unique<zmq::pollitem_t>();
    camera->poll_ptr->socket = *camera->socket_ptr;
    camera->poll_ptr->fd = 0;
    camera->poll_ptr->events = ZMQ_POLLIN;
    camera->poll_ptr->revents = 0;
    camera->port = this->asset_manager_->GetFreePort();

    std::string camera_uri =
        "tcp://" + 
        this->anx_ip_ +
        ":"
        + std::to_string(camera->port);
    ROS_INFO("%s uri: %s", camera->name.c_str(), camera_uri.c_str());

    camera->socket_ptr->connect(camera_uri);
    camera->socket_ptr->set(zmq::sockopt::subscribe, ""); 

    image_transport::ImageTransport it(*camera->nh_ptr);
    camera->publisher = it.advertiseCamera("image_raw", 1);
  }

  // Starting camera threads
  for(auto& camera : this->camera_){
    camera.thread_ptr = std::make_unique<std::thread>(
        &CameraManager::CameraThread, this, &camera
    );
  }
}

void CameraManager::Stop(){
  if(this->started_){
    this->started_ = false;
  }else{
    return;
  }
  for(auto& camera : this->camera_){
    camera.thread_ptr->join();

    if(camera.streaming){
      nlohmann::json stop_msg_json;
      stop_msg_json["asset"] = {
        {"type", "camera"},
        {"id", camera.select.id}
      };
      if(this->asset_manager_->StopAsset(stop_msg_json)){
        camera.streaming = false;
        ROS_INFO("%s stopped!", camera.name.c_str());
      }else{
        ROS_INFO("Failed to stop %s!", camera.name.c_str());
      }
    }

    this->asset_manager_->ReturnFreePort(camera.port);
  }
  this->camera_.clear();
}

void CameraManager::OnStateChange(nlohmann::json state){
  /* ROS_INFO(state.dump().c_str()); // Debug */
  for(auto& camera : this->camera_){
    if(this->IsPresent(camera, state)){
      if(camera.streaming){
        // Do Nothing
        /* ROS_INFO("IsPresent and Streaming"); // Debug */
      }else{
        /* ROS_INFO("IsPresent and Not Streaming"); // Debug */
        // Start Streaming
        nlohmann::json start_msg_json;
        start_msg_json["asset"] = {
          {"type", "camera"},
          {"meta", {
                     {"id", camera.select.id},
                     {"stream", {
                                  {"fps", camera.select.stream.fps},
                                  {"width", camera.select.stream.width},
                                  {"height", camera.select.stream.height},
                                  {"pixel_format", camera.select.stream.pixel_format}
                                }
                     },
                     {"compression_quality", camera.select.compression_quality}
                   }
          }
        };
        start_msg_json["port"] = {
          {"pub", camera.port}
        };
        /* ROS_INFO(start_msg_json.dump().c_str()); // Debug */

        if(this->asset_manager_->StartAsset(start_msg_json)){
          ROS_INFO("%s started!", camera.name.c_str());
          camera.streaming = true;
        }else{
          ROS_INFO("Failed to start %s!", camera.name.c_str());
        }
      }
    }else{
      if(camera.streaming){
        /* ROS_INFO("IsNotPresent and Streaming"); // Debug */
        // Stop Streaming
        camera.streaming = false;
      }else{
        /* ROS_INFO("IsNotPresent and Not Streaming"); // Debug */
        // Do Nothing
      }
    }
  }
}

void CameraManager::CameraThread(Camera* camera){
  ROS_INFO("%s thread listening to %d started!", camera->name.c_str(), camera->port);
  while (this->started_ && ros::ok()) {
    zmq::message_t msg;
    zmq::poll(camera->poll_ptr.get(), 1, 100);

    if (camera->poll_ptr->revents & ZMQ_POLLIN){
      try{
        camera->socket_ptr->recv(msg);
      }catch (std::exception& e){
        ROS_INFO("Connection to %s terminated!", camera->name.c_str());
        break;
      }

      /* ROS_INFO("%s: %d", camera->name.c_str(), msg.size()); // Debug */
      this->PublishCameraStream(msg, camera);
    }

  }
}

bool CameraManager::IsPresent(Camera& camera, nlohmann::json& state){
  for(int i=0; i<state.size(); i++){
    /* ROS_INFO(state[i].dump().c_str()); // Debug */
    if(state[i]["id"] == camera.select.id){
      // Check if requested stream and compression_quality is available
      for(int j=0; j<state[i]["stream"].size(); j++){
        if(
            (state[i]["stream"][j]["fps"] == camera.select.stream.fps) &&
            (state[i]["stream"][j]["width"] == camera.select.stream.width) &&
            (state[i]["stream"][j]["height"] == camera.select.stream.height) &&
            (state[i]["stream"][j]["pixel_format"] == camera.select.stream.pixel_format) &&
            std::find(
              state[i]["compression_quality"].begin(),
              state[i]["compression_quality"].end(),
              camera.select.compression_quality
            ) != state[i]["compression_quality"].end()){
          return true;
        }
      }
    }
  }
  return false;
}

void CameraManager::PublishCameraStream(zmq::message_t& msg, Camera* camera){
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header.seq = camera->seq;
  cv_ptr->header.frame_id = camera->frame_id;
  cv_ptr->header.stamp = ros::Time::now();

  // Decode color/mono image
  try{
    uint32_t size = msg.size();
    auto img_raw = (BYTE *) msg.data();
    std::vector<BYTE> bytes(img_raw, img_raw + size);
    cv_ptr->image = cv::imdecode(
        cv::Mat(/* base64_decode(jpeg_img.to_string()) */bytes),
        cv::IMREAD_UNCHANGED
    );

    switch (cv_ptr->image.channels()){
      case 1:
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
        break;
      case 3:
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        break;
      default:
        ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
        break;
    }
  }catch (cv::Exception& e){
    ROS_ERROR("%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0)){
    sensor_msgs::CameraInfo camera_info = camera->camera_info_manager_ptr->getCameraInfo();
    camera_info.header = cv_ptr->header;

    camera->publisher.publish(*cv_ptr->toImageMsg(), camera_info);
  }
}
