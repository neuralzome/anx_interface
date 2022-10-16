#include "hermes_interface/usb_serial_manager.h"

UsbSerialManager::UsbSerialManager(AssetManagerInterface* asset_manager)
  : started_(false){
  // Save pointer to asset_manager 
  this->asset_manager_ = asset_manager;

  ros::NodeHandle nh_private("~");
  nh_private.getParam("hermes_ip", this->hermes_ip_);
}

void UsbSerialManager::Start(){
  if(this->started_){
    return;
  }else{
    this->started_ = true;
  }
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("usb_serial")){
    return;
  }
  // Populate usb_serial_ from parameter server
  XmlRpc::XmlRpcValue usb_serial_params;
  nh_private.getParam("usb_serial", usb_serial_params);
  for(int i=0; i<usb_serial_params.size(); i++){
    this->usb_serial_.emplace_back();
    UsbSerial* usb_serial = &this->usb_serial_.back();

    usb_serial->name = std::string(usb_serial_params[i]["name"]);
    usb_serial->select.id = std::string(usb_serial_params[i]["select"]["id"]);
    usb_serial->select.baud = int(usb_serial_params[i]["select"]["baud"]);
    usb_serial->select.delimiter = std::string(usb_serial_params[i]["select"]["delimiter"]);

    usb_serial->pub_socket_ptr = std::make_unique<zmq::socket_t>(
        this->ctx_,
        zmq::socket_type::sub
    );
    usb_serial->pub_poll_ptr = std::make_unique<zmq::pollitem_t>();
    usb_serial->pub_poll_ptr->socket = *usb_serial->pub_socket_ptr;
    usb_serial->pub_poll_ptr->fd = 0;
    usb_serial->pub_poll_ptr->events = ZMQ_POLLIN;
    usb_serial->pub_poll_ptr->revents = 0;

    usb_serial->sub_socket_ptr = std::make_unique<zmq::socket_t>(
        this->ctx_,
        zmq::socket_type::pub
    );

    usb_serial->pub_port = this->asset_manager_->GetFreePort();
    usb_serial->sub_port = this->asset_manager_->GetFreePort();

    std::string pub_usb_serial_uri =
        "tcp://" + 
        this->hermes_ip_ +
        ":"
        + std::to_string(usb_serial->pub_port);
    ROS_INFO("%s pub uri: %s", usb_serial->name.c_str(), pub_usb_serial_uri.c_str());

    std::string sub_usb_serial_uri =
        "tcp://" + 
        std::string("*") +
        ":"
        + std::to_string(usb_serial->sub_port);
    ROS_INFO("%s sub uri: %s", usb_serial->name.c_str(), sub_usb_serial_uri.c_str());

    usb_serial->pub_socket_ptr->connect(pub_usb_serial_uri);
    usb_serial->pub_socket_ptr->set(zmq::sockopt::subscribe, ""); 

    usb_serial->sub_socket_ptr->bind(sub_usb_serial_uri);

    usb_serial->publisher = usb_serial->nh.advertise<std_msgs::String>(usb_serial->name + "/out", 10); 

  }

  // Starting usb_serial threads
  for(auto& usb_serial : this->usb_serial_){
    usb_serial.subscriber = usb_serial.nh.subscribe<std_msgs::String>(
        usb_serial.name + "/in",
        10,
        boost::bind(
          &UsbSerialManager::ToUsbSerialCb,
          this,
          _1, &usb_serial)
        );
    usb_serial.pub_thread_ptr = std::make_unique<std::thread>(
        &UsbSerialManager::FromUsbSerialThread, this, &usb_serial
    );
  }
}

void UsbSerialManager::Stop(){
  if(this->started_){
    this->started_ = false;
  }else{
    return;
  }
  for(auto& usb_serial : this->usb_serial_){

    usb_serial.pub_thread_ptr->join();

    if(usb_serial.streaming){
      nlohmann::json stop_msg_json;
      stop_msg_json["asset"] = {
        {"type", "usb_serial"},
        {"id", usb_serial.select.id}
      };
      if(this->asset_manager_->StopAsset(stop_msg_json)){
        usb_serial.streaming = false;
        ROS_INFO("%s stopped!", usb_serial.name.c_str());
      }else{
        ROS_INFO("Failed to stop %s!", usb_serial.name.c_str());
      }
    }

    this->asset_manager_->ReturnFreePort(usb_serial.pub_port);
    this->asset_manager_->ReturnFreePort(usb_serial.sub_port);
  }
  this->usb_serial_.clear();
}

void UsbSerialManager::OnStateChange(nlohmann::json state){
  /* ROS_INFO(state.dump().c_str()); // Debug */
  for(auto& usb_serial : this->usb_serial_){
    if(this->IsPresent(usb_serial, state)){
      if(usb_serial.streaming){
        // Do Nothing
        /* ROS_INFO("IsPresent and Streaming"); // Debug */
      }else{
        /* ROS_INFO("IsPresent and Not Streaming"); // Debug */
        // Start Streaming
        nlohmann::json start_msg_json;
        start_msg_json["asset"] = {
          {"type", "usb_serial"},
          {"meta", {
                     {"baud", usb_serial.select.baud},
                     {"delimiter", usb_serial.select.delimiter},
                     {"id", usb_serial.select.id}
                   }
          }
        };
        start_msg_json["port"] = {
          {"pub", usb_serial.pub_port},
          {"sub", usb_serial.sub_port}
        };
        /* ROS_INFO(start_msg_json.dump().c_str()); // Debug */

        if(this->asset_manager_->StartAsset(start_msg_json)){
          ROS_INFO("%s started!", usb_serial.name.c_str());
          usb_serial.streaming = true;
        }else{
          ROS_INFO("Failed to start %s!", usb_serial.name.c_str());
        }
      }
    }else{
      if(usb_serial.streaming){
        /* ROS_INFO("IsNotPresent and Streaming"); // Debug */
        // Stop Streaming
        usb_serial.streaming = false;
      }else{
        /* ROS_INFO("IsNotPresent and Not Streaming"); // Debug */
        // Do Nothing
      }
    }
  }
}

void UsbSerialManager::ToUsbSerialCb(
    const std_msgs::String::ConstPtr& usb_serial_ros_msg_ptr,
    UsbSerial* usb_serial){
  if(!usb_serial->streaming){
    return;
  }
  /* ROS_INFO("msg: %s sent!", usb_serial_ros_msg_ptr->data.c_str()); // Debug */
  nlohmann::json msg_json = {
    {"data", usb_serial_ros_msg_ptr->data}
  };

  try{
    usb_serial->sub_socket_ptr->send(
        zmq::buffer(msg_json.dump()),
        zmq::send_flags::dontwait
    );
  }catch (std::exception& e){
  }
}

void UsbSerialManager::FromUsbSerialThread(UsbSerial* usb_serial){
  ROS_INFO("%s thread listening to %d started!", usb_serial->name.c_str(), usb_serial->pub_port);
  while (this->started_ && ros::ok()) {
    zmq::message_t msg;
    zmq::poll(usb_serial->pub_poll_ptr.get(), 1, 100);

    if (usb_serial->pub_poll_ptr->revents & ZMQ_POLLIN){
      try{
        usb_serial->pub_socket_ptr->recv(msg);
      }catch (std::exception& e){
        ROS_INFO("Connection to %s terminated!", usb_serial->name.c_str());
        break;
      }

      /* ROS_INFO("%s: %s", usb_serial->name.c_str(), msg.to_string().c_str()); // Debug */
      try{
        nlohmann::json msg_json = nlohmann::json::parse(msg.to_string());
        std_msgs::String usb_serial_ros_msg;
        usb_serial_ros_msg.data = msg_json["data"];
        usb_serial->publisher.publish(usb_serial_ros_msg);
      }catch(std::exception& e){
        ROS_ERROR("Invalid msg received!");
        ROS_ERROR("msg [InUsbSerialThread]: %s", msg.to_string().c_str());
      }
    }
  }
}

bool UsbSerialManager::IsPresent(UsbSerial& usb_serial, nlohmann::json& state){
  for(int i=0; i<state.size(); i++){
    /* ROS_INFO(state[i].dump().c_str()); // Debug */
    if(
        (state[i]["id"] == usb_serial.select.id) &&
        std::find(
          state[i]["baud"].begin(),
          state[i]["baud"].end(),
          usb_serial.select.baud
        ) != state[i]["baud"].end() &&
        std::find(
          state[i]["delimiter"].begin(),
          state[i]["delimiter"].end(),
          usb_serial.select.delimiter
        ) != state[i]["delimiter"].end()){
      /* ROS_INFO(state[i].dump().c_str()); // Debug */
      return true;
    }else{
    }
  }
  return false;
}
