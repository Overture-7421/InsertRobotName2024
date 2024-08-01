// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/SuperStructure/SuperStructure.h"

#include <frc/filter/LinearFilter.h>
#include <frc2/command/SubsystemBase.h>
#include <units/acceleration.h>

class ChassisAccelToStructureFF  : public frc2::SubsystemBase {
public:
  using FactorUnits = units::compound_unit<units::volt, units::inverse<units::meters_per_second_squared>>;
  ChassisAccelToStructureFF(Chassis* chassis, SuperStructure* superStructure);
  void Periodic() override;
  void shuffleboardPeriodic();
private:
  Chassis* chassis = nullptr;
  SuperStructure* superStructure = nullptr;
  units::meters_per_second_squared_t maxAccel = 0_mps_sq, filteredAccel = 0_mps_sq;
  double accelToVoltsFactor = 0.0;
  units::volt_t outFF = 0_V;
  frc::LinearFilter<units::meters_per_second_squared_t> accelXFilter = frc::LinearFilter<units::meters_per_second_squared_t>::SinglePoleIIR(0.05, 0.02_s);
};
