#include "anx_interface/phone_manager.h"

PhoneManager::PhoneManager(AssetManagerInterface* asset_manager){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  if(!nh_private.getParam("anx_ip", this->anx_ip_)){
    this->anx_ip_ = "localhost";
  }
}

void PhoneManager::Start(){
  this->phone_.socket_ptr = std::make_unique<zmq::socket_t>(
      this->ctx_,
      zmq::socket_type::sub
  );
  this->phone_.poll_ptr = std::make_unique<zmq::pollitem_t>();
  this->phone_.poll_ptr->socket = *this->phone_.socket_ptr;
  this->phone_.poll_ptr->fd = 0;
  this->phone_.poll_ptr->events = ZMQ_POLLIN;
  this->phone_.poll_ptr->revents = 0;
  this->phone_.port = this->asset_manager_->GetFreePort();

  std::string phone_uri =
      "tcp://" + 
      this->anx_ip_ +
      ":"
      + std::to_string(this->phone_.port);
  ROS_INFO("phone uri: %s", phone_uri.c_str());

  this->phone_.socket_ptr->connect(phone_uri);
  this->phone_.socket_ptr->set(zmq::sockopt::subscribe, ""); 

  this->phone_.publisher = this->phone_.nh.advertise<anx_interface_msgs::PhoneState>("phone", 10); 

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
        {"meta", {{"id", this->phone_.id}}}
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
    zmq::poll(this->phone_.poll_ptr.get(), 1, 100);
    if (this->phone_.poll_ptr->revents & ZMQ_POLLIN){
      this->phone_.socket_ptr->recv(msg);

      /* ROS_INFO("phone: %s", msg.to_string().c_str()); // Debug */
      try{
        nlohmann::json msg_json = nlohmann::json::parse(msg.to_string());
        anx_interface_msgs::PhoneState phone_state_ros_msg;

        phone_state_ros_msg.header.seq = this->phone_.seq++;
        phone_state_ros_msg.header.stamp = ros::Time::now();
        phone_state_ros_msg.header.frame_id = "phone";

        phone_state_ros_msg.uptime = msg_json["uptime"];

        phone_state_ros_msg.ram.used = msg_json["ram"]["used"];
        phone_state_ros_msg.ram.total = msg_json["ram"]["total"];

        phone_state_ros_msg.vram.used = msg_json["vram"]["used"];
        phone_state_ros_msg.vram.total = msg_json["vram"]["total"];

        phone_state_ros_msg.storage.used = msg_json["storage"]["used"];
        phone_state_ros_msg.storage.total = msg_json["storage"]["total"];

        std::vector<anx_interface_msgs::CpuFreq> freqs;
        for(int i=0; i<msg_json["cpu"]["freq"].size(); i++){
          anx_interface_msgs::CpuFreq freq; 
          freq.current = msg_json["cpu"]["freq"][i]["current"];
          freq.min = msg_json["cpu"]["freq"][i]["min"];
          freq.max = msg_json["cpu"]["freq"][i]["max"];
          freqs.emplace_back(freq);
        }
        phone_state_ros_msg.cpu.freq = freqs;

        phone_state_ros_msg.cpu.processor = msg_json["cpu"]["processor"];
        phone_state_ros_msg.cpu.architectures = msg_json["cpu"]["architectures"];
        phone_state_ros_msg.cpu.type = msg_json["cpu"]["type"];

        phone_state_ros_msg.gpu.renderer = msg_json["gpu"]["renderer"];

        std::vector<anx_interface_msgs::Thermal> thermals;
        for(int i=0; i<msg_json["thermals"].size(); i++){
          anx_interface_msgs::Thermal thermal;
          thermal.name = msg_json["thermal"][i]["name"];
          thermal.temp = msg_json["thermal"][i]["temp"];
          thermals.emplace_back(thermal);
        }
        phone_state_ros_msg.thermals = thermals;
        
        phone_state_ros_msg.battery.charging = msg_json["battery"]["charging"];
        phone_state_ros_msg.battery.level = msg_json["battery"]["level"];
        phone_state_ros_msg.battery.current = msg_json["battery"]["current"];
        phone_state_ros_msg.battery.voltage = msg_json["battery"]["voltage"];

        this->phone_.publisher.publish(phone_state_ros_msg);
      }catch (std::exception& e){
        ROS_ERROR("Invalid msg received!");
        ROS_ERROR("msg [PhoneThread]: %s", msg.to_string().c_str());
      }
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
