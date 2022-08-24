#include "hermes_interface/asset_manager.h"

AssetManager::AssetManager():
    sub_asset_state_socket_(sub_asset_state_ctx_, zmq::socket_type::req),
    asset_state_socket_(asset_state_ctx_, zmq::socket_type::sub),
    start_asset_state_socket_(start_asset_state_ctx_, zmq::socket_type::req),
    stop_asset_state_socket_(stop_asset_state_ctx_, zmq::socket_type::req),
    subscribed_(false){

  ros::NodeHandle nh_private("~");

  nh_private.getParam("hermes_ip", this->hermes_ip_);
  nh_private.getParam("subscribe_asset_port", this->subscribe_asset_port_);
  nh_private.getParam("start_asset_port", this->stop_asset_port_);
  nh_private.getParam("stop_asset_port", this->stop_asset_port_);
  nh_private.getParam("asset_state_port", this->asset_state_port_);

  std::string asset_state_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->asset_state_port_);
  asset_state_socket_.connect(asset_state_uri);
  ROS_INFO("asset_state_uri: %s", asset_state_uri.c_str());
  asset_state_socket_.set(zmq::sockopt::subscribe, "");

  std::string subscribe_asset_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->subscribe_asset_port_);
  ROS_INFO("sub_asset_state_uri: %s", subscribe_asset_uri.c_str());
  this->sub_asset_state_socket_.connect(subscribe_asset_uri);
}

void AssetManager::Start(){
  this->asset_state_thread_ = std::make_unique<std::thread>(
      &AssetManager::AssetStateThread, this
  );

  ROS_INFO("Subscribing...");
  if(this->Subscribe(true)){
    ROS_INFO("Subscribed to asset state stream");
    this->subscribed_ = true;
  }else{
    if(this->Subscribe(false)){
      if(this->Subscribe(true)){
        ROS_INFO("Subscribed to asset state stream.");
        this->subscribed_ = true;
      }else{
        ROS_INFO("Failed to subscribe to asset state stream.");
      }
    }else{
      ROS_INFO("Failed to subscribe to asset state stream.");
    }
  }
}

void AssetManager::Stop(){
  if(this->subscribed_){
    if(this->Subscribe(false)){
      ROS_INFO("Unsubscribed to asset state stream.");
      this->subscribed_ = false;
    }else{
      ROS_INFO("Failed to Unsubscribe to asset state stream.");
    }
  }
}

void AssetManager::AssetStateThread(){
  while (ros::ok()) {
    zmq::message_t msg;
    this->asset_state_socket_.recv(msg);

    ROS_INFO(msg.to_string().c_str());
  }
}

bool AssetManager::Subscribe(bool subscribe){
  nlohmann::json msg_req_json;
  msg_req_json["subscribe"] = subscribe;
  /* ROS_INFO(msg_req_json.dump().c_str()); // Debug */

  this->sub_asset_state_socket_.send(zmq::buffer(msg_req_json.dump()), zmq::send_flags::dontwait);

  zmq::message_t msg_res;
  this->sub_asset_state_socket_.recv(msg_res);
  /* ROS_INFO(msg_res.to_string().c_str()); // Debug */
  
  try{
    nlohmann::json msg_res_json = nlohmann::json::parse(msg_res.to_string());
    return msg_res_json["success"];
  }catch (int err){
    ROS_ERROR("Invalid msg received!");
    ROS_ERROR("msg: %s", msg_res.to_string().c_str());
    return false;
  }
}
