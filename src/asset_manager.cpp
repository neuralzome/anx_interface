#include "hermes_interface/asset_manager.h"
#include "hermes_interface/hermes_interface.h"
#include "hermes_interface/imu_manager.h"
#include "hermes_interface/usb_serial_manager.h"

AssetManager::AssetManager():
    imu_manager_(this),
    usb_serial_manager_(this),
    sub_asset_state_socket_(sub_asset_state_ctx_, zmq::socket_type::req),
    asset_state_socket_(asset_state_ctx_, zmq::socket_type::sub),
    start_asset_socket_(start_asset_ctx_, zmq::socket_type::req),
    stop_asset_socket_(stop_asset_ctx_, zmq::socket_type::req),
    subscribed_(false){

  ros::NodeHandle nh_private("~");

  nh_private.getParam("hermes_ip", this->hermes_ip_);
  nh_private.getParam("subscribe_asset_port", this->subscribe_asset_port_);
  nh_private.getParam("start_asset_port", this->start_asset_port_);
  nh_private.getParam("stop_asset_port", this->stop_asset_port_);
  nh_private.getParam("asset_state_port", this->asset_state_port_);

  // Initialize port pool
  std::vector<int> port_pool_range;
  nh_private.getParam("port_pool_range", port_pool_range);
  assert(port_pool_range.size() == 2);
  assert(port_pool_range[1] > port_pool_range[0]);
  for(int i=port_pool_range[0]; i<=port_pool_range[1]; i++){
    this->port_pool_.emplace_back(i);
  }

  // Initialize socket for listning to asset stream
  std::string asset_state_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->asset_state_port_);
  asset_state_socket_.connect(asset_state_uri);
  ROS_INFO("asset_state_uri: %s", asset_state_uri.c_str());
  asset_state_socket_.set(zmq::sockopt::subscribe, "");

  // Initialize socket for subscribing to asset stream
  std::string subscribe_asset_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->subscribe_asset_port_);
  ROS_INFO("sub_asset_state_uri: %s", subscribe_asset_uri.c_str());
  this->sub_asset_state_socket_.connect(subscribe_asset_uri);

  // Initialize socket for starting asset stream
  std::string start_asset_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->start_asset_port_);
  ROS_INFO("start_state_uri: %s", start_asset_uri.c_str());
  this->start_asset_socket_.connect(start_asset_uri);
  
  // Initialize socket for stopping asset stream
  std::string stop_asset_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->stop_asset_port_);
  ROS_INFO("stop_state_uri: %s", stop_asset_uri.c_str());
  this->stop_asset_socket_.connect(stop_asset_uri);
}

void AssetManager::Start(){
  // Start Assets
  this->imu_manager_.Start();
  this->usb_serial_manager_.Start();

  // Start listning to asset stream
  this->asset_state_thread_ = std::make_unique<std::thread>(
      &AssetManager::AssetStateThread, this
  );

  // Subscribe to asset stream
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

int AssetManager::GetFreePort(){
  int port = this->port_pool_.back();
  this->port_pool_.pop_back();
  return port;
}

void AssetManager::Stop(){
  // Unsubscribe to asset stream
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

    ROS_INFO(msg.to_string().c_str()); // Debug
    try{
      nlohmann::json msg_json = nlohmann::json::parse(msg.to_string());
      this->imu_manager_.OnStateChange(msg_json["imu"]);
      this->usb_serial_manager_.OnStateChange(msg_json["usb_serial"]);
    }catch (std::exception& e){
      ROS_ERROR("Invalid msg received!");
      ROS_ERROR("msg [AssetStateThread]: %s", msg.to_string().c_str());
    }
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
    if(!msg_res_json["success"]){
      ROS_INFO(msg_res_json["message"].dump().c_str());
    }
    return msg_res_json["success"];
  }catch (std::exception& e){
    ROS_ERROR("Invalid msg received!");
    ROS_ERROR("msg [Subscribe]: %s", msg_res.to_string().c_str());
    return false;
  }
}

bool AssetManager::StartAsset(nlohmann::json msg){
  this->start_asset_socket_.send(zmq::buffer(msg.dump()), zmq::send_flags::dontwait);

  zmq::message_t msg_res;
  this->start_asset_socket_.recv(msg_res);
  /* ROS_INFO(msg_res.to_string().c_str()); // Debug */
  
  try{
    nlohmann::json msg_res_json = nlohmann::json::parse(msg_res.to_string());
    if(!msg_res_json["success"]){
      ROS_INFO(msg_res_json["message"].dump().c_str());
    }
    return msg_res_json["success"];
  }catch (std::exception& e){
    ROS_ERROR("Invalid msg received!");
    ROS_ERROR("msg [StartAsset]: %s", msg_res.to_string().c_str());
    return false;
  }
}

bool AssetManager::StopAsset(nlohmann::json msg){
  this->stop_asset_socket_.send(zmq::buffer(msg.dump()), zmq::send_flags::dontwait);

  zmq::message_t msg_res;
  this->stop_asset_socket_.recv(msg_res);
  /* ROS_INFO(msg_res.to_string().c_str()); // Debug */
  
  try{
    nlohmann::json msg_res_json = nlohmann::json::parse(msg_res.to_string());
    if(!msg_res_json["success"]){
      ROS_INFO(msg_res_json["message"].dump().c_str());
    }
    return msg_res_json["success"];
  }catch (std::exception& e){
    ROS_ERROR("Invalid msg received!");
    ROS_ERROR("msg [StopAsset]: %s", msg_res.to_string().c_str());
    return false;
  }
}
