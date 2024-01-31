// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterCommand.h"

ShooterCommand::ShooterCommand(Shooter* m_Shooter, double m_velocity) {
  // Use addRequirements() here to declare subsystem dependencies.

  this->m_velocity = m_velocity;
  this->m_Shooter = m_Shooter;
  AddRequirements({ m_Shooter });
}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {
    m_Shooter->setVelocityVoltage(m_velocity);
}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  
double error = abs(m_velocity - m_Shooter->getCurrentVelocity());

  if(error <= 10){
    return true;
  }
  else{
    return false;
  }
}
