#include "anx_interface/imu_manager.h"

ImuManager::ImuManager(AssetManagerInterface* asset_manager): started_(false){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  if(!nh_private.getParam("anx_ip", this->anx_ip_)){
    this->anx_ip_ = "localhost";
  }
}

std::string ImuManager::Name(){
  return "imu";
}

bool ImuManager::IsCore(){
  return false;
}

void ImuManager::Start(){
  if(this->started_){
    return;
  }else{
    this->started_ = true;
  }
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("imu")){
    return;
  }
  // Populate imu_ from parameter server
  XmlRpc::XmlRpcValue imu_params;
  nh_private.getParam("imu", imu_params);
  for(int i=0; i<imu_params.size(); i++){
    this->imu_.emplace_back();
    Imu* imu = &this->imu_.back();

    imu->name = std::string(imu_params[i]["name"]);
    imu->select.id = std::string(imu_params[i]["select"]["id"]);
    imu->select.fps = int(imu_params[i]["select"]["fps"]);
    
    for(int j=0; j<3; j++){
      imu->covariance.orientation_covariance_diagonal[j]
          = double(imu_params[i]["covariance"]["orientation_covariance_diagonal"][j]);
    }
    for(int j=0; j<3; j++){
      imu->covariance.angular_velocity_covariance_diagonal[j]
          = double(imu_params[i]["covariance"]["angular_velocity_covariance_diagonal"][j]);
    }
    for(int j=0; j<3; j++){
      imu->covariance.linear_acceleration_covariance_diagonal[j]
          = double(imu_params[i]["covariance"]["linear_acceleration_covariance_diagonal"][j]);
    }

    imu->socket_ptr = std::make_unique<zmq::socket_t>(
        this->ctx_,
        zmq::socket_type::sub
    );
    imu->poll_ptr = std::make_unique<zmq::pollitem_t>();
    imu->poll_ptr->socket = *imu->socket_ptr;
    imu->poll_ptr->fd = 0;
    imu->poll_ptr->events = ZMQ_POLLIN;
    imu->poll_ptr->revents = 0;

    imu->port = this->asset_manager_->GetFreePort();

    std::string imu_uri =
        "tcp://" + 
        this->anx_ip_ +
        ":"
        + std::to_string(imu->port);
    ROS_INFO("%s uri: %s", imu->name.c_str(), imu_uri.c_str());

    imu->socket_ptr->connect(imu_uri);
    imu->socket_ptr->set(zmq::sockopt::subscribe, ""); 

    imu->publisher = this->imu_.back().nh.advertise<sensor_msgs::Imu>(this->imu_.back().name, 10); 
    imu->raw_publisher = this->imu_.back().nh.advertise<sensor_msgs::Imu>(
      this->imu_.back().name + "/raw", 10);
    imu->magnetic_field_publisher = this->imu_.back().nh.advertise<sensor_msgs::MagneticField>(
      this->imu_.back().name + "/raw/magnetometer", 10);
  }

  for(auto& imu : this->imu_){
    imu.thread_ptr = std::make_unique<std::thread>(
        &ImuManager::ImuThread, this, &imu
    );
  }
}

void ImuManager::Stop(){
  if(this->started_){
    this->started_ = false;
  }else{
    return;
  }
  for(auto& imu : this->imu_){

    imu.thread_ptr->join();

    if(imu.streaming){
      nlohmann::json stop_msg_json;
      stop_msg_json["asset"] = {
        {"type", "imu"},
        {"id", imu.select.id}
      };
      if(this->asset_manager_->StopAsset(stop_msg_json)){
        imu.streaming = false;
        ROS_INFO("%s stopped!", imu.name.c_str());
      }else{
        ROS_INFO("Failed to stop %s!", imu.name.c_str());
      }
    }

    this->asset_manager_->ReturnFreePort(imu.port);
  }
  this->imu_.clear();
}

void ImuManager::OnStateChange(nlohmann::json state){
  /* ROS_INFO(state.dump().c_str()); // Debug */
  for(auto& imu : this->imu_){
    if(this->IsPresent(imu, state)){
      if(imu.streaming){
        // Do Nothing
        /* ROS_INFO("IsPresent and Streaming"); // Debug */
      }else{
        /* ROS_INFO("IsPresent and Not Streaming"); // Debug */
        // Start Streaming
        nlohmann::json start_msg_json;
        start_msg_json["asset"] = {
          {"type", "imu"},
          {"meta", {{"fps", imu.select.fps}, {"id", imu.select.id}}}
        };
        start_msg_json["port"] = {
          {"pub", imu.port}
        };
        /* ROS_INFO(start_msg_json.dump().c_str()); // Debug */

        if(this->asset_manager_->StartAsset(start_msg_json)){
          ROS_INFO("%s started!", imu.name.c_str());
          imu.streaming = true;
        }else{
          ROS_INFO("Failed to start %s!", imu.name.c_str());
        }
      }
    }else{
      if(imu.streaming){
        /* ROS_INFO("IsNotPresent and Streaming"); // Debug */
        // Stop Streaming
        imu.streaming = false;
      }else{
        /* ROS_INFO("IsNotPresent and Not Streaming"); // Debug */
        // Do Nothing
      }
    }
  }
}

void ImuManager::ImuThread(Imu* imu){
  ROS_INFO("%s thread listening to %d started!", imu->name.c_str(), imu->port);
  while (this->started_ && ros::ok()) {
    zmq::message_t msg;
    zmq::poll(imu->poll_ptr.get(), 1, 100);

    if (imu->poll_ptr->revents & ZMQ_POLLIN){
      try{
        imu->socket_ptr->recv(msg);
      }catch (std::exception& e){
        ROS_INFO("Connection to %s terminated!", imu->name.c_str());
        break;
      }

      /* ROS_INFO("%s: %s", imu->name.c_str(), msg.to_string().c_str()); // Debug */
      try{
        nlohmann::json imu_msg_json = nlohmann::json::parse(msg.to_string());
        nlohmann::json filtered_msg_json = imu_msg_json["filtered"];
        nlohmann::json raw_msg_json = imu_msg_json["raw"];

        sensor_msgs::Imu imu_ros_msg;
        sensor_msgs::Imu raw_imu_ros_msg;
        sensor_msgs::MagneticField magnetic_field_ros_msg;

        imu_ros_msg.header.seq = imu->seq++;
        imu_ros_msg.header.stamp = ros::Time::now();
        imu_ros_msg.header.frame_id = imu->name;
        
        raw_imu_ros_msg.header.seq = imu->seq++;
        raw_imu_ros_msg.header.stamp = ros::Time::now();
        raw_imu_ros_msg.header.frame_id = imu->name;

        magnetic_field_ros_msg.header.seq = imu->seq++;
        magnetic_field_ros_msg.header.stamp = ros::Time::now();
        magnetic_field_ros_msg.header.frame_id = imu->name;

        imu_ros_msg.orientation.x = filtered_msg_json["q"][0];
        imu_ros_msg.orientation.y = filtered_msg_json["q"][1];
        imu_ros_msg.orientation.z = filtered_msg_json["q"][2];
        imu_ros_msg.orientation.w = filtered_msg_json["q"][3];
        imu_ros_msg.orientation_covariance[0] = imu->covariance.orientation_covariance_diagonal[0];
        imu_ros_msg.orientation_covariance[4] = imu->covariance.orientation_covariance_diagonal[1];
        imu_ros_msg.orientation_covariance[8] = imu->covariance.orientation_covariance_diagonal[2];

        imu_ros_msg.angular_velocity.x = filtered_msg_json["w"][0];
        imu_ros_msg.angular_velocity.y = filtered_msg_json["w"][1];
        imu_ros_msg.angular_velocity.z = filtered_msg_json["w"][2];
        imu_ros_msg.angular_velocity_covariance[0] = imu->covariance.angular_velocity_covariance_diagonal[0];
        imu_ros_msg.angular_velocity_covariance[4] = imu->covariance.angular_velocity_covariance_diagonal[1];
        imu_ros_msg.angular_velocity_covariance[8] = imu->covariance.angular_velocity_covariance_diagonal[2];

        imu_ros_msg.linear_acceleration.x = filtered_msg_json["a"][0];
        imu_ros_msg.linear_acceleration.y = filtered_msg_json["a"][1];
        imu_ros_msg.linear_acceleration.z = filtered_msg_json["a"][2];
        imu_ros_msg.linear_acceleration_covariance[0] = imu->covariance.linear_acceleration_covariance_diagonal[0];
        imu_ros_msg.linear_acceleration_covariance[4] = imu->covariance.linear_acceleration_covariance_diagonal[1];
        imu_ros_msg.linear_acceleration_covariance[8] = imu->covariance.linear_acceleration_covariance_diagonal[2];


        // populate raw msg
        raw_imu_ros_msg.angular_velocity.x = raw_msg_json["w"][0];
        raw_imu_ros_msg.angular_velocity.y = raw_msg_json["w"][1];
        raw_imu_ros_msg.angular_velocity.z = raw_msg_json["w"][2];

        raw_imu_ros_msg.linear_acceleration.x = raw_msg_json["a"][0];
        raw_imu_ros_msg.linear_acceleration.y = raw_msg_json["a"][1];
        raw_imu_ros_msg.linear_acceleration.z = raw_msg_json["a"][2];

        // populate magnetic field
        magnetic_field_ros_msg.magnetic_field.x = raw_msg_json["mu"][0];
        magnetic_field_ros_msg.magnetic_field.y = raw_msg_json["mu"][1];
        magnetic_field_ros_msg.magnetic_field.z = raw_msg_json["mu"][2];

        imu->publisher.publish(imu_ros_msg);
        imu->raw_publisher.publish(raw_imu_ros_msg);
        imu->magnetic_field_publisher.publish(magnetic_field_ros_msg);

      }catch (std::exception& e){
        ROS_ERROR("Invalid msg received!");
        ROS_ERROR("msg [ImuThread]: %s", msg.to_string().c_str());
      }
    }
  }
}

bool ImuManager::IsPresent(Imu& imu, nlohmann::json& state){
  for(int i=0; i<state.size(); i++){
    /* ROS_INFO(state[i].dump().c_str()); // Debug */
    if(state[i]["id"] == imu.select.id){
      if(
          std::find(
            state[i]["fps"].begin(),
            state[i]["fps"].end(),
            imu.select.fps
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
