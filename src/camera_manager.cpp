#include "hermes_interface/camera_manager.h"

CameraManager::CameraManager(AssetManagerInterface* asset_manager){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  nh_private.getParam("hermes_ip", this->hermes_ip_);

  // Populate camera_ from parameter server
  XmlRpc::XmlRpcValue camera_params;
  nh_private.getParam("camera", camera_params);
  for(int i=0; i<camera_params.size(); i++){
    this->camera_.emplace_back();

    this->camera_.back().name = std::string(camera_params[i]["name"]);
    this->camera_.back().select.id = std::string(camera_params[i]["select"]["id"]);
    this->camera_.back().select.stream.fps = int(camera_params[i]["select"]["stream"]["fps"]);
    this->camera_.back().select.stream.width = int(camera_params[i]["select"]["stream"]["width"]);
    this->camera_.back().select.stream.height = int(camera_params[i]["select"]["stream"]["height"]);
    this->camera_.back().select.stream.pixel_format = std::string(camera_params[i]["select"]["stream"]["pixel_format"]);
    this->camera_.back().select.compression_quality = int(camera_params[i]["select"]["compression_quality"]);

    this->camera_.back().camera_info_uri = std::string(camera_params[i]["camera_info_uri"]);
  }
}

void CameraManager::Start(){
  // Create sockets and thread for camera and start listning to ports for camera stream
  for(auto& camera : this->camera_){
    camera.ctx_ptr = std::make_unique<zmq::context_t>();
    camera.socket_ptr = std::make_unique<zmq::socket_t>(
        *camera.ctx_ptr,
        zmq::socket_type::sub
    );
    camera.port = this->asset_manager_->GetFreePort();

    std::string camera_uri =
        "tcp://" + 
        this->hermes_ip_ +
        ":"
        + std::to_string(camera.port);
    ROS_INFO("%s uri: %s", camera.name.c_str(), camera_uri.c_str());

    camera.socket_ptr->connect(camera_uri);
    camera.socket_ptr->set(zmq::sockopt::subscribe, ""); 

    camera.thread_ptr = std::make_unique<std::thread>(
        &CameraManager::CameraThread, this, &camera
    );
  }
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
  while (ros::ok()) {
    zmq::message_t msg;
    camera->socket_ptr->recv(msg);

    ROS_INFO("%s: %s", camera->name.c_str(), msg.to_string().c_str());
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
