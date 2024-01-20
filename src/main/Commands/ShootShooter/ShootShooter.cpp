// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShootShooter.h"

#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>

ShootShooter::ShootShooter(Shooter* m_Shooter, double m_velocity) {
  // Use addRequirements() here to declare subsystem dependencies.

  this->m_velocity = m_velocity;
  this->m_Shooter = m_Shooter;
  AddRequirements({ m_Shooter });
}

// Called when the command is initially scheduled.
void ShootShooter::Initialize() {

  m_Shooter->setVelocityVoltage(m_velocity);

}

// Called repeatedly when this Command is scheduled to run
void ShootShooter::Execute() {
}

// Called once the command ends or is interrupted.
void ShootShooter::End(bool interrupted) {
  m_Shooter->setVelocityVoltage(0.0);
}

// Returns true when the command should end.
bool ShootShooter::IsFinished() {
  return false;
}
