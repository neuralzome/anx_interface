#pragma once

#include <string>

#include <nlohmann/json.hpp>

class AssetInterface{
public:
  virtual std::string Name() = 0;
  virtual bool IsCore() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual void OnStateChange(nlohmann::json state) = 0;
};
