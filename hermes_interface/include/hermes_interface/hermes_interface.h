#pragma once

#include <memory>
#include <signal.h>
#include <iostream>

#include <ros/ros.h>

#include "hermes_interface/asset_manager.h"

class HermesInterface{
public:
  static HermesInterface* GetInstance();
  void Start();
private:
  HermesInterface();
  static void SigintHandler(int sig);

  ros::NodeHandle nh_;
  static std::unique_ptr<HermesInterface> instance_;
  
  std::unique_ptr<AssetManager> asset_manager_ptr_;
};
