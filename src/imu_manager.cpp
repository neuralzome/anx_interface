#include "hermes_interface/imu_manager.h"

ImuManager::ImuManager(AssetManagerInterface* asset_manager){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  nh_private.getParam("hermes_ip", this->hermes_ip_);

  // Populate imu_ from parameter server
  XmlRpc::XmlRpcValue imu_params;
  nh_private.getParam("imu", imu_params);
  for(int i=0; i<imu_params.size(); i++){
    this->imu_.emplace_back();

    this->imu_.back().name = std::string(imu_params[i]["name"]);
    this->imu_.back().select.id = std::string(imu_params[i]["select"]["id"]);
    this->imu_.back().select.fps = int(imu_params[i]["select"]["fps"]);
    
    for(int j=0; j<3; j++){
      this->imu_.back().covariance.orientation_covariance_diagonal[j]
          = double(imu_params[i]["covariance"]["orientation_covariance_diagonal"][j]);
    }
    for(int j=0; j<3; j++){
      this->imu_.back().covariance.angular_velocity_covariance_diagonal[j]
          = double(imu_params[i]["covariance"]["angular_velocity_covariance_diagonal"][j]);
    }
    for(int j=0; j<3; j++){
      this->imu_.back().covariance.linear_acceleration_covariance_diagonal[j]
          = double(imu_params[i]["covariance"]["linear_acceleration_covariance_diagonal"][j]);
    }
    this->imu_.back().port = this->asset_manager_->GetFreePort();

    this->imu_.back().ctx_ptr = std::make_unique<zmq::context_t>();
    this->imu_.back().socket_ptr = std::make_unique<zmq::socket_t>(
        *this->imu_.back().ctx_ptr,
        zmq::socket_type::sub
    );
    std::string imu_uri =
        "tcp://" + 
        this->hermes_ip_ +
        ":"
        + std::to_string(this->imu_.back().port);
    ROS_INFO("%s uri: %s", this->imu_.back().name.c_str(), imu_uri.c_str());

    this->imu_.back().socket_ptr->connect(imu_uri);
    this->imu_.back().socket_ptr->set(zmq::sockopt::subscribe, ""); 
  }
}

void ImuManager::Start(){
  // Create thread and start listning to ports for imu stream
  for(auto& imu : this->imu_){
    imu.thread_ptr = std::make_unique<std::thread>(
        &ImuManager::ImuThread, this, &imu
    );
  }
}

void ImuManager::OnStateChange(nlohmann::json state){
  for(auto& imu : this->imu_){
    if(this->IsPresent(imu, state)){
      if(imu.streaming){
        // Do Nothing
      }else{
        // Start Streaming
        nlohmann::json start_msg_json;
        start_msg_json["asset"] = {
          {"type", "imu"},
          {"meta", {{"fps", imu.select.fps}, {"id", imu.select.id}}}
        };
        start_msg_json["port"] = {"pub", imu.port};

        if(this->asset_manager_->StartAsset(start_msg_json)){
          ROS_INFO("%s started!", imu.name.c_str());
          imu.streaming = true;
        }else{
          ROS_INFO("Failed to start %s!", imu.name.c_str());
        }
      }
    }else{
      if(imu.streaming){
        // Stop Streaming
        imu.streaming = false;
      }else{
        // Do Nothing
      }
    }
  }
}

void ImuManager::ImuThread(Imu* imu){
  while (ros::ok()) {
    zmq::message_t msg;
    imu->socket_ptr->recv(msg);

    ROS_INFO("%s: %s", imu->name.c_str(), msg.to_string().c_str());
  }
}

bool ImuManager::IsPresent(Imu& imu, nlohmann::json& state){
  for(int i=0; i<state.size(); i++){
    if(state[i]["id"] == imu.select.id){
      if(
          std::find(
            std::begin(state[i]["fps"]),
            std::end(state[i]["fps"]),
            imu.select.fps
          ) == std::end(state[i]["fps"])
      ){
        return true;
      }
    }
  }
  return false;
}
