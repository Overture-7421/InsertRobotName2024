// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ServoDashboard.h"
#include "frc/smartdashboard/SmartDashboard.h"

ServoDashboard::ServoDashboard(SupportArms* supportArms) {
  this->supportArms = supportArms;
  AddRequirements({ supportArms });

};

// Called when the command is initially scheduled.
void ServoDashboard::Initialize() {
  frc::SmartDashboard::PutNumber("Servo", 0.0);
}

// Called repeatedly when this Command is scheduled to run
void ServoDashboard::Execute() {
  double servoNumber = frc::SmartDashboard::GetNumber("Servo", 0.0);
  supportArms->SetAngle(servoNumber);
}

// Called once the command ends or is interrupted.
void ServoDashboard::End(bool interrupted) {}

// Returns true when the command should end.
bool ServoDashboard::IsFinished() {
  return false;
}
