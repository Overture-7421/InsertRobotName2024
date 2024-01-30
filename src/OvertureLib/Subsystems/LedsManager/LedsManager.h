// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>

class LedsManager : public frc2::SubsystemBase {
 public:
  LedsManager(int pwmPort, int ledLength);

  virtual void UpdateLeds(std::vector<frc::AddressableLED::LEDData>& ledBuffer) = 0;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() final;
  
 private:
  frc::AddressableLED ledStrip;
  std::vector<frc::AddressableLED::LEDData> ledBuffer;
};
