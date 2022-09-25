#include "hermes_interface/hermes_interface.h"
#include "hermes_interface/asset_manager.h"

std::unique_ptr<HermesInterface> HermesInterface::instance_ = nullptr;

HermesInterface* HermesInterface::GetInstance(){
  if(HermesInterface::instance_ == nullptr){
    instance_ = std::unique_ptr<HermesInterface>(new HermesInterface());

    signal(SIGINT, HermesInterface::SigintHandler);

    // Construct HermesInterface
    HermesInterface::instance_->asset_manager_ptr_ = std::make_unique<AssetManager>();
  }
  return HermesInterface::instance_.get();
}

void HermesInterface::Start(){
  HermesInterface::instance_->asset_manager_ptr_->Start();
}

HermesInterface::HermesInterface(){
}

void HermesInterface::SigintHandler(int sig){
  HermesInterface::instance_->asset_manager_ptr_->Stop();
  ros::shutdown();
}

