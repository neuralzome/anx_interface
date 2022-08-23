#include <ros/ros.h>
#include "hermes_interface/asset_manager.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "hermes_interface");
  ros::NodeHandle nh;

  AssetManager asset_manager;
  asset_manager.Start();

  ros::spin();
  return 0;
}
