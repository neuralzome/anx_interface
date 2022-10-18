#include "hermes_interface/speaker_manager.h"

SpeakerManager::SpeakerManager(AssetManagerInterface* asset_manager)
  : started_(false){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  nh_private.getParam("hermes_ip", this->hermes_ip_);

}

void SpeakerManager::Start(){
  if(this->started_){
    return;
  }else{
    this->started_ = true;
  }
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("speaker")){
    return;
  }
  // Populate usb_serial_ from parameter server
  XmlRpc::XmlRpcValue speaker_params;
  nh_private.getParam("speaker", speaker_params);
  for(int i=0; i<speaker_params.size(); i++){
    this->speaker_.emplace_back();
    Speaker* speaker = &this->speaker_.back();

    speaker->name = std::string(speaker_params[i]["name"]);
    speaker->select.id = std::string(speaker_params[i]["select"]["id"]);
    speaker->select.language = std::string(speaker_params[i]["select"]["language"]);

    speaker->sub_socket_ptr = std::make_unique<zmq::socket_t>(
        this->ctx_,
        zmq::socket_type::pub
    );

    speaker->sub_port = this->asset_manager_->GetFreePort();

    std::string sub_speaker_uri =
        "tcp://" + 
        std::string("*") +
        ":"
        + std::to_string(speaker->sub_port);
    ROS_INFO("%s sub uri: %s", speaker->name.c_str(), sub_speaker_uri.c_str());

    speaker->sub_socket_ptr->bind(sub_speaker_uri);

    speaker->subscriber = speaker->nh.subscribe<std_msgs::String>(
        speaker->name,
        10,
        boost::bind(
          &SpeakerManager::ToSpeakerCb,
          this,
          _1, speaker)
        );
  }
}

void SpeakerManager::Stop(){
  if(this->started_){
    this->started_ = false;
  }else{
    return;
  }
  for(auto& speaker : this->speaker_){
    if(speaker.streaming){
      nlohmann::json stop_msg_json;
      stop_msg_json["asset"] = {
        {"type", "speaker"},
        {"id", speaker.select.id}
      };
      if(this->asset_manager_->StopAsset(stop_msg_json)){
        speaker.streaming = false;
        ROS_INFO("%s stopped!", speaker.name.c_str());
      }else{
        ROS_INFO("Failed to stop %s!", speaker.name.c_str());
      }
    }

    this->asset_manager_->ReturnFreePort(speaker.sub_port);
  }
  this->speaker_.clear();
}

void SpeakerManager::OnStateChange(nlohmann::json state){
  /* ROS_INFO(state.dump().c_str()); // Debug */
  for(auto& speaker : this->speaker_){
    if(this->IsPresent(speaker, state)){
      if(speaker.streaming){
        // Do Nothing
        /* ROS_INFO("IsPresent and Streaming"); // Debug */
      }else{
        /* ROS_INFO("IsPresent and Not Streaming"); // Debug */
        // Start Streaming
        nlohmann::json start_msg_json;
        start_msg_json["asset"] = {
          {"type", "speaker"},
          {"meta", {
                     {"language", speaker.select.language},
                     {"id", speaker.select.id}
                   }
          }
        };
        start_msg_json["port"] = {
          {"sub", speaker.sub_port}
        };
        /* ROS_INFO(start_msg_json.dump().c_str()); // Debug */

        if(this->asset_manager_->StartAsset(start_msg_json)){
          ROS_INFO("%s started!", speaker.name.c_str());
          speaker.streaming = true;
        }else{
          ROS_INFO("Failed to start %s!", speaker.name.c_str());
        }
      }
    }else{
      if(speaker.streaming){
        /* ROS_INFO("IsNotPresent and Streaming"); // Debug */
        // Stop Streaming
        speaker.streaming = false;
      }else{
        /* ROS_INFO("IsNotPresent and Not Streaming"); // Debug */
        // Do Nothing
      }
    }
  }
}

void SpeakerManager::ToSpeakerCb(
    const std_msgs::String::ConstPtr& speaker_msg_ptr,
    Speaker* speaker){
  if(!speaker->streaming){
    return;
  }
  /* ROS_INFO("msg: %s sent!", speaker_msg_ptr->data.c_str()); // Debug */
  nlohmann::json msg_json = {
    {"data", speaker_msg_ptr->data}
  };

  try{
    speaker->sub_socket_ptr->send(
        zmq::buffer(msg_json.dump()),
        zmq::send_flags::dontwait
    );
  }catch (std::exception& e){
  }
}

bool SpeakerManager::IsPresent(Speaker& speaker, nlohmann::json& state){
  for(int i=0; i<state.size(); i++){
    /* ROS_INFO(state[i].dump().c_str()); // Debug */
    if(
        (state[i]["id"] == speaker.select.id) &&
        std::find(
          state[i]["language"].begin(),
          state[i]["language"].end(),
          speaker.select.language
        ) != state[i]["language"].end()){
      /* ROS_INFO(state[i].dump().c_str()); // Debug */
      return true;
    }else{
    }
  }
  return false;
}
