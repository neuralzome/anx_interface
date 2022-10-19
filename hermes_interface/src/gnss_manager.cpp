#include "hermes_interface/gnss_manager.h"

GnssManager::GnssManager(AssetManagerInterface* asset_manager): started_(false){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  nh_private.getParam("hermes_ip", this->hermes_ip_);
}

void GnssManager::Start(){
  if(this->started_){
    return;
  }else{
    this->started_ = true;
  }
  ros::NodeHandle nh_private("~");

  // Populate gnss_ from parameter server
  XmlRpc::XmlRpcValue gnss_params;
  nh_private.getParam("gnss", gnss_params);
  for(int i=0; i<gnss_params.size(); i++){
    this->gnss_.emplace_back();
    Gnss* gnss = &this->gnss_.back();

    gnss->name = std::string(gnss_params[i]["name"]);
    gnss->select.id = std::string(gnss_params[i]["select"]["id"]);
    gnss->select.fps = int(gnss_params[i]["select"]["fps"]);

    gnss->socket_ptr = std::make_unique<zmq::socket_t>(
        this->ctx_,
        zmq::socket_type::sub
    );
    gnss->poll_ptr = std::make_unique<zmq::pollitem_t>();
    gnss->poll_ptr->socket = *gnss->socket_ptr;
    gnss->poll_ptr->fd = 0;
    gnss->poll_ptr->events = ZMQ_POLLIN;
    gnss->poll_ptr->revents = 0;

    gnss->port = this->asset_manager_->GetFreePort();

    std::string gnss_uri =
        "tcp://" + 
        this->hermes_ip_ +
        ":"
        + std::to_string(gnss->port);
    ROS_INFO("%s uri: %s", gnss->name.c_str(), gnss_uri.c_str());

    gnss->socket_ptr->connect(gnss_uri);
    gnss->socket_ptr->set(zmq::sockopt::subscribe, ""); 

    gnss->publisher = this->gnss_.back().nh.advertise<nmea_msgs::Sentence>(this->gnss_.back().name, 10); 
  }

  for(auto& gnss : this->gnss_){
    gnss.thread_ptr = std::make_unique<std::thread>(
        &GnssManager::GnssThread, this, &gnss
    );
  }
}

void GnssManager::Stop(){
  if(this->started_){
    this->started_ = false;
  }else{
    return;
  }
  for(auto& gnss : this->gnss_){

    gnss.thread_ptr->join();

    if(gnss.streaming){
      nlohmann::json stop_msg_json;
      stop_msg_json["asset"] = {
        {"type", "gnss"},
        {"id", gnss.select.id}
      };
      if(this->asset_manager_->StopAsset(stop_msg_json)){
        gnss.streaming = false;
        ROS_INFO("%s stopped!", gnss.name.c_str());
      }else{
        ROS_INFO("Failed to stop %s!", gnss.name.c_str());
      }
    }

    this->asset_manager_->ReturnFreePort(gnss.port);
  }
  this->gnss_.clear();
}

void GnssManager::OnStateChange(nlohmann::json state){
  /* ROS_INFO(state.dump().c_str()); // Debug */
  for(auto& gnss : this->gnss_){
    if(this->IsPresent(gnss, state)){
      if(gnss.streaming){
        // Do Nothing
        /* ROS_INFO("IsPresent and Streaming"); // Debug */
      }else{
        /* ROS_INFO("IsPresent and Not Streaming"); // Debug */
        // Start Streaming
        nlohmann::json start_msg_json;
        start_msg_json["asset"] = {
          {"type", "gnss"},
          {"meta", {{"fps", gnss.select.fps}, {"id", gnss.select.id}}}
        };
        start_msg_json["port"] = {
          {"pub", gnss.port}
        };
        /* ROS_INFO(start_msg_json.dump().c_str()); // Debug */

        if(this->asset_manager_->StartAsset(start_msg_json)){
          ROS_INFO("%s started!", gnss.name.c_str());
          gnss.streaming = true;
        }else{
          ROS_INFO("Failed to start %s!", gnss.name.c_str());
        }
      }
    }else{
      if(gnss.streaming){
        /* ROS_INFO("IsNotPresent and Streaming"); // Debug */
        // Stop Streaming
        gnss.streaming = false;
      }else{
        /* ROS_INFO("IsNotPresent and Not Streaming"); // Debug */
        // Do Nothing
      }
    }
  }
}

void GnssManager::GnssThread(Gnss* gnss){
  ROS_INFO("%s thread listening to %d started!", gnss->name.c_str(), gnss->port);
  while (this->started_ && ros::ok()) {
    zmq::message_t msg;
    zmq::poll(gnss->poll_ptr.get(), 1, 100);

    if (gnss->poll_ptr->revents & ZMQ_POLLIN){
      try{
        gnss->socket_ptr->recv(msg);
      }catch (std::exception& e){
        ROS_INFO("Connection to %s terminated!", gnss->name.c_str());
        break;
      }

      /* ROS_INFO("%s: %s", gnss->name.c_str(), msg.to_string().c_str()); // Debug */
      try{
        nlohmann::json msg_json = nlohmann::json::parse(msg.to_string());
        // publish nmea string to topic
        nmea_msgs::Sentence nmea_msg;
        nmea_msg.header.seq = gnss->seq++;
        nmea_msg.header.stamp = ros::Time::now();
        nmea_msg.header.frame_id = gnss->name;

        nmea_msg.sentence = msg_json["nmea"];

        gnss->publisher.publish(nmea_msg);

      }catch (std::exception& e){
        ROS_ERROR("Invalid msg received!");
        ROS_ERROR("msg [GnssThread]: %s", msg.to_string().c_str());
      }
    }
  }
}

bool GnssManager::IsPresent(Gnss& gnss, nlohmann::json& state){
  for(int i=0; i<state.size(); i++){
    /* ROS_INFO(state[i].dump().c_str()); // Debug */
    if(state[i]["id"] == gnss.select.id){
      if(
          std::find(
            state[i]["fps"].begin(),
            state[i]["fps"].end(),
            gnss.select.fps
          ) != state[i]["fps"].end()
      ){
        return true;
      }
    }else{
      /* ROS_INFO(state[i]["id"].dump().c_str()); // Debug */
    }
  }
  return false;
}
