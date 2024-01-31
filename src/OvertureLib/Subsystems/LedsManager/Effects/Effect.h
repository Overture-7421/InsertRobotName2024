#pragma once
#include "OvertureLib/Subsystems/LedsManager/LedsManager.h"

class LedCommand { 
  public:
  virtual void Apply(LedStrip* ledStrip) = 0;
};