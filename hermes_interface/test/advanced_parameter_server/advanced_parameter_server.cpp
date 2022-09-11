#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "advanced_parameter_server");
  ros::NodeHandle nh;
  ROS_INFO("Begin!");


  ros::NodeHandle nh_private("~");
  XmlRpc::XmlRpcValue imu;
  nh_private.getParam("imu", imu);
  std::stringstream msg;

  if(imu.getType() == XmlRpc::XmlRpcValue::Type::TypeArray && imu.size() > 0){
    for(int i=0; i<imu.size(); i++){
      msg<<imu[i]["name"]<<": "<<imu[i]["select"]["id"]<<std::endl;
    }
  }

  ROS_INFO(msg.str().c_str());

  ROS_INFO("End!");
  ros::spin();
  return 0;
}
