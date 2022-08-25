#pragma once

#include <nlohmann/json.hpp>

class AssetManagerInterface{
public:
  virtual int GetFreePort() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual bool StartAsset(nlohmann::json msg) = 0;
  virtual bool StopAsset(nlohmann::json msg) = 0;
};
