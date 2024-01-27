// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterCommand.h"

ShooterCommand::ShooterCommand(Shooter* shooter, frc::XboxController* joystick): m_shooter(shooter), m_joystick(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
  this->m_shooter = m_shooter;

}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {
  double reverse = Utils::ApplyAxisFilter(m_joystick->GetLeftTriggerAxis(),0.3);
  double take = Utils::ApplyAxisFilter(m_joystick->GetRightTriggerAxis(),0.3);

  if (take > reverse) {
    m_shooter->setVoltage(1.0_V);
  } else if (take < reverse) {
    m_shooter->setVoltage(1.0_V);
  } else {
    m_shooter->setVoltage(0_V);
  }

}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {
  m_shooter->setVoltage(0_V);
}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  return false;
}
