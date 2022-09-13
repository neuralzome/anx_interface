#include "hermes_interface/hermes_interface.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "hermes_interface", ros::init_options::NoSigintHandler);

  HermesInterface* hermes_interface_ptr = HermesInterface::GetInstance();
  hermes_interface_ptr->Start();

  ros::spin();
  return 0;
}
