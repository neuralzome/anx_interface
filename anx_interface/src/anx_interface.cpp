#include "anx_interface/anx_interface.h"
#include "anx_interface/asset_manager.h"

std::unique_ptr<AnxInterface> AnxInterface::instance_ = nullptr;

AnxInterface* AnxInterface::GetInstance(){
  if(AnxInterface::instance_ == nullptr){
    instance_ = std::unique_ptr<AnxInterface>(new AnxInterface());

    signal(SIGINT, AnxInterface::SigintHandler);

    // Construct AnxInterface
    AnxInterface::instance_->asset_manager_ptr_ = std::make_unique<AssetManager>();
  }
  return AnxInterface::instance_.get();
}

void AnxInterface::Start(){
  AnxInterface::instance_->asset_manager_ptr_->Start();
}

AnxInterface::AnxInterface(){
}

void AnxInterface::SigintHandler(int sig){
  AnxInterface::instance_->asset_manager_ptr_->Stop();
  ros::shutdown();
}

