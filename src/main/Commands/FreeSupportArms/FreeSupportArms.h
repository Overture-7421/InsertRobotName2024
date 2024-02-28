// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "main/Subsystems/SupportArms/SupportArms.h"

class FreeSupportArms
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 FreeSupportArms> {
 public:
  FreeSupportArms(SupportArms* supportArms, double angle);

  void Initialize() override;
  bool IsFinished() override;

  private:
  SupportArms* supportArms;
  double angle;
};