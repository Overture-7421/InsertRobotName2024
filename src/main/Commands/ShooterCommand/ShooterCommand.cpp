// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterCommand.h"

ShooterCommand::ShooterCommand(Shooter* shooter, double velocity) {
  this->velocity = velocity;
  this->shooter = shooter;
  AddRequirements({ shooter });
}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {
    shooter->setVelocityVoltage(velocity);
}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  
double upperError = abs(velocity - shooter->getUpperMotorCurrentVelocity());
double lowerError = abs(velocity - shooter->getLowerMotorCurrentVelocity());
  if(upperError <= 10 && lowerError  <= 10){
    return true;
  } else {
    return false;
  }
}
