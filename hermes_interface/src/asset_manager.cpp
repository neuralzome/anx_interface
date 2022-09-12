#include "hermes_interface/asset_manager.h"
#include "hermes_interface/camera_manager.h"
#include "hermes_interface/hermes_interface.h"
#include "hermes_interface/imu_manager.h"
#include "hermes_interface/phone_manager.h"
#include "hermes_interface/usb_serial_manager.h"
#include <zmq.hpp>

AssetManager::AssetManager():
    imu_manager_(this),
    usb_serial_manager_(this),
    camera_manager_(this),
    phone_manager_(this),
    sub_asset_state_socket_(sub_asset_state_ctx_, zmq::socket_type::req),
    asset_state_socket_(asset_state_ctx_, zmq::socket_type::sub),
    start_asset_socket_(start_asset_ctx_, zmq::socket_type::req),
    stop_asset_socket_(stop_asset_ctx_, zmq::socket_type::req),
    get_identity_socket_(get_identity_ctx_, zmq::socket_type::req),
    send_signal_socket_(send_signal_ctx_, zmq::socket_type::req),
    subscribed_(false),
    non_core_asset_started_(false){

  ros::NodeHandle nh_private("~");

  this->start_server_ = nh_.advertiseService(
      "start_non_core_assets",
      &AssetManager::StartNonCoreAssetsCb,
      this
  );

  this->signal_server_ = nh_.advertiseService(
      "signal",
      &AssetManager::SignalCb,
      this
  );

  nh_private.getParam("hermes_ip", this->hermes_ip_);
  nh_private.getParam("linux_ip", this->linux_ip_);
  nh_private.getParam("subscribe_asset_port", this->subscribe_asset_port_);
  nh_private.getParam("start_asset_port", this->start_asset_port_);
  nh_private.getParam("stop_asset_port", this->stop_asset_port_);
  nh_private.getParam("asset_state_port", this->asset_state_port_);
  nh_private.getParam("get_identity_port", this->get_identity_port_);
  nh_private.getParam("send_signal_port", this->send_signal_port_);

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
  
  // Initialize socket for getting identity
  std::string get_identity_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->get_identity_port_);
  ROS_INFO("get_identity_uri: %s", get_identity_uri.c_str());
  this->get_identity_socket_.connect(get_identity_uri);
  
  // Initialize socket for sending signal
  std::string send_signal_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->send_signal_port_);
  ROS_INFO("send_signal_uri: %s", send_signal_uri.c_str());
  this->send_signal_socket_.connect(send_signal_uri);
}

void AssetManager::Start(){
  // Start listning to asset stream
  this->asset_state_thread_ = std::make_unique<std::thread>(
      &AssetManager::AssetStateThread, this
  );

  // Start core assets
  this->phone_manager_.Start();

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

  this->nh_.setParam("identity", this->GetIdentity());
}

int AssetManager::GetFreePort(){
  int port = this->port_pool_.back();
  this->port_pool_.pop_back();
  return port;
}

void AssetManager::ReturnFreePort(int port){
  this->port_pool_.emplace_back(port);
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

    this->asset_state_ = msg.to_string();
    ROS_INFO(this->asset_state_.c_str()); // Debug
    try{
      nlohmann::json msg_json = nlohmann::json::parse(msg.to_string());
      this->phone_manager_.OnStateChange(msg_json["phone"]);
      if(this->non_core_asset_started_){
        this->imu_manager_.OnStateChange(msg_json["imu"]);
        this->usb_serial_manager_.OnStateChange(msg_json["usb_serial"]);
        this->camera_manager_.OnStateChange(msg_json["camera"]);
      }
    }catch (std::exception& e){
      ROS_ERROR("Invalid msg received!");
      ROS_ERROR("msg [AssetStateThread]: %s", msg.to_string().c_str());
    }
  }
}

bool AssetManager::Subscribe(bool subscribe){
  nlohmann::json msg_req_json;
  msg_req_json["subscribe"] = subscribe;
  msg_req_json["subscribers_ip"] = this->linux_ip_; 
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

bool AssetManager::StartNonCoreAssetsCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
  if(req.data){
    if(this->non_core_asset_started_){
      res.success = true;
      res.message = "Already started!";
    }else{
      this->imu_manager_.Start();
      this->usb_serial_manager_.Start();
      this->camera_manager_.Start();

      if(!this->asset_state_.empty()){
        try{
          nlohmann::json msg_json = nlohmann::json::parse(this->asset_state_);
          this->imu_manager_.OnStateChange(msg_json["imu"]);
          this->usb_serial_manager_.OnStateChange(msg_json["usb_serial"]);
          this->camera_manager_.OnStateChange(msg_json["camera"]);
        }catch (std::exception& e){
          ROS_ERROR("Invalid msg received!");
          ROS_ERROR("msg [AssetStateThread]: %s", this->asset_state_.c_str());
        }
      }

      res.success = true;
      this->non_core_asset_started_ = true;
    }
  }else{
    if(this->non_core_asset_started_){
      this->imu_manager_.Stop();
      this->usb_serial_manager_.Stop();
      this->camera_manager_.Stop();
      res.success = true;
      this->non_core_asset_started_ = false;
    }else{
      res.success = true;
      res.message = "Already stopped!";
    }
  }
  return true;
}


std::string AssetManager::GetIdentity(){
  std::string identity = "";
  this->get_identity_socket_.send(zmq::buffer("{}"), zmq::send_flags::dontwait);

  zmq::message_t msg_res;
  this->get_identity_socket_.recv(msg_res);
  /* ROS_INFO(msg_res.to_string().c_str()); // Debug */
  
  try{
    nlohmann::json msg_res_json = nlohmann::json::parse(msg_res.to_string());
    identity = msg_res_json["imei"];
  }catch (std::exception& e){
    ROS_ERROR("Invalid msg received!");
    ROS_ERROR("msg [GetIdentity]: %s", msg_res.to_string().c_str());
  }

  return identity;
}

bool AssetManager::SignalCb(
    hermes_interface_msgs::SendSignal::Request  &req,
    hermes_interface_msgs::SendSignal::Response &res
){
  switch(req.signal.signal){
    case hermes_interface_msgs::Signal::SHUTDOWN:
      // TODO: Send SHUTDOWN signal
      nlohmann::json msg_req_json;
      msg_req_json["signal"] = 0;

      this->send_signal_socket_.send(zmq::buffer(msg_req_json.dump()), zmq::send_flags::dontwait);

      zmq::message_t msg_res;
      this->sub_asset_state_socket_.recv(msg_res);
      /* ROS_INFO(msg_res.to_string().c_str()); // Debug */
      
      try{
        nlohmann::json msg_res_json = nlohmann::json::parse(msg_res.to_string());
        if(!msg_res_json["success"]){
          res.success = true;
        }else{
          res.success = false;
          res.message = "Invalid signal";
        }
      }catch (std::exception& e){
        ROS_ERROR("Invalid msg received!");
        ROS_ERROR("msg [SendSignal]: %s", msg_res.to_string().c_str());
        res.success = false;
        res.message = "Invalid msg received!";
      }
      break;
  }
  return true;
}
