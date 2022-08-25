#include <string>
#include <sstream>
#include <vector>

#include <nlohmann/json.hpp>

#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "json_parsing");
  ros::NodeHandle nh;
  ROS_INFO("Begin!");


  ros::NodeHandle nh_private("~");
  std::stringstream msg;

 int i = 3; 
  nlohmann::json msg_json = {
    {"data", {1,2,3,4,5,6}}
  };
  msg<<msg_json.dump()<<std::endl;
  if(
      std::find(
        msg_json["data"].begin(),
        msg_json["data"].end(),
        i
      ) != msg_json["data"].end()
  ){
    msg<<i<<std::endl;
  }


  ROS_INFO(msg.str().c_str());

  ROS_INFO("End!");
  ros::spin();
  return 0;
}
