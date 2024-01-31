// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArmCommand.h"

SupportArmCommand::SupportArmCommand(SupportArms* supportArms, SupportArmsState targetState) {
  this->supportArms = supportArms;
  this->targetState = targetState;
  AddRequirements(supportArms);
}

void SupportArmCommand::Initialize() {
  	supportArms->setTargetCoord(targetState);
}

void SupportArmCommand::Execute() {}

void SupportArmCommand::End(bool interrupted) {}

bool SupportArmCommand::IsFinished() {
  double error = abs(targetState.lowerAngle - supportArms->getLowerAngle());

  if (error <= 2 ) {
	  return true;
	} else {
    return false;
  }
}
