#pragma once

#include <memory>
#include <signal.h>
#include <iostream>

#include <ros/ros.h>

#include "anx_interface/asset_manager.h"

class AnxInterface{
public:
  static AnxInterface* GetInstance();
  void Start();
private:
  AnxInterface();
  static void SigintHandler(int sig);

  ros::NodeHandle nh_;
  static std::unique_ptr<AnxInterface> instance_;
  
  std::unique_ptr<AssetManager> asset_manager_ptr_;
};
