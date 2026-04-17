#pragma once

#include "motion_control/controller/ControllerConfig.h"

class ControllerBackend {
 public:
  virtual ~ControllerBackend() = default;

  virtual bool configure(const ControllerConfig& config) = 0;
  virtual void launch() = 0;
  virtual const char* name() const = 0;
};
