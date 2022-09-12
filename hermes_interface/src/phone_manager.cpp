#include "hermes_interface/phone_manager.h"
#include "hermes_interface_msgs/PhoneState.h"

PhoneManager::PhoneManager(AssetManagerInterface* asset_manager){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  nh_private.getParam("hermes_ip", this->hermes_ip_);
}

void PhoneManager::Start(){
  ros::NodeHandle nh_private("~");

  XmlRpc::XmlRpcValue phone_params;
  nh_private.getParam("phone", phone_params);
  this->phone_.ctx_ptr = std::make_unique<zmq::context_t>();
  this->phone_.socket_ptr = std::make_unique<zmq::socket_t>(
      *this->phone_.ctx_ptr,
      zmq::socket_type::sub
  );
  this->phone_.port = this->asset_manager_->GetFreePort();

  std::string phone_uri =
      "tcp://" + 
      this->hermes_ip_ +
      ":"
      + std::to_string(this->phone_.port);
  ROS_INFO("phone uri: %s", phone_uri.c_str());

  this->phone_.socket_ptr->connect(phone_uri);
  this->phone_.socket_ptr->set(zmq::sockopt::subscribe, ""); 

  this->phone_.publisher = this->phone_.nh.advertise<hermes_interface_msgs::PhoneState>("phone", 10); 

  this->phone_.thread_ptr = std::make_unique<std::thread>(
      &PhoneManager::PhoneThread, this
  );
}

void PhoneManager::OnStateChange(nlohmann::json state){
  /* ROS_INFO(state.dump().c_str()); // Debug */
  if(this->IsPresent(state)){
    if(this->phone_.streaming){
      // Do Nothing
      /* ROS_INFO("IsPresent and Streaming"); // Debug */
    }else{
      /* ROS_INFO("IsPresent and Not Streaming"); // Debug */
      // Start Streaming
      nlohmann::json start_msg_json;
      start_msg_json["asset"] = {
        {"type", "phone"},
        {"meta", {"id", this->phone_.id}}
      };
      start_msg_json["port"] = {
        {"pub", this->phone_.port}
      };
      /* ROS_INFO(start_msg_json.dump().c_str()); // Debug */

      if(this->asset_manager_->StartAsset(start_msg_json)){
        ROS_INFO("phone started!");
        this->phone_.streaming = true;
      }else{
        ROS_INFO("Failed to start phone!");
      }
    }
  }else{
    if(this->phone_.streaming){
      /* ROS_INFO("IsNotPresent and Streaming"); // Debug */
      // Stop Streaming
      this->phone_.streaming = false;
    }else{
      /* ROS_INFO("IsNotPresent and Not Streaming"); // Debug */
      // Do Nothing
    }
  }
}

void PhoneManager::PhoneThread(){
  ROS_INFO("phone thread listening to %d started!", this->phone_.port);
  while (ros::ok()) {
    zmq::message_t msg;
    this->phone_.socket_ptr->recv(msg);

    /* ROS_INFO("phone: %s", msg.to_string().c_str()); // Debug */
    try{
      nlohmann::json msg_json = nlohmann::json::parse(msg.to_string());
      hermes_interface_msgs::PhoneState phone_state_ros_msg;

      phone_state_ros_msg.header.seq = this->phone_.seq++;
      phone_state_ros_msg.header.stamp = ros::Time::now();
      phone_state_ros_msg.header.frame_id = "phone";

      phone_state_ros_msg.charging = msg_json["charging"];
      phone_state_ros_msg.cpu_ram_usage = msg_json["cpu_ram_usage"];
      phone_state_ros_msg.cpu_usage = msg_json["cpu_usage"];
      phone_state_ros_msg.cpu_temp = msg_json["cpu_temp"];
      phone_state_ros_msg.gpu_vram_usage = msg_json["gpu_vram_usage"];
      phone_state_ros_msg.gpu_usage = msg_json["gpu_usage"];

      this->phone_.publisher.publish(phone_state_ros_msg);
    }catch (std::exception& e){
      ROS_ERROR("Invalid msg received!");
      ROS_ERROR("msg [PhoneThread]: %s", msg.to_string().c_str());
    }
  }
}

bool PhoneManager::IsPresent(nlohmann::json& state){
  if(state["id"] == this->phone_.id){
    return true;
  }else{
    return false;
  }
}
