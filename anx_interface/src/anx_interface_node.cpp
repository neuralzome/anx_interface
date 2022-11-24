#include "anx_interface/anx_interface.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "anx_interface", ros::init_options::NoSigintHandler);

  AnxInterface* anx_interface_ptr = AnxInterface::GetInstance();
  anx_interface_ptr->Start();

  ros::spin();
  return 0;
}
