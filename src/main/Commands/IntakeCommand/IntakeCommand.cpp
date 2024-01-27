// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeCommand.h"
#include "main/Commands/IntakeCommand/IntakeCommand.h"

IntakeCommand::IntakeCommand(Intake* intake, frc::XboxController* joystick): m_intake(intake), m_joystick(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
    this->m_intake = m_intake;

}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {
  double reverse = Utils::ApplyAxisFilter(m_joystick->GetLeftTriggerAxis(),0.3);
  double take = Utils::ApplyAxisFilter(m_joystick->GetRightTriggerAxis(),0.3);

  if (take > reverse) {
    m_intake->setVoltage(-1.0_V);
  } else if (take < reverse) {
    m_intake->setVoltage(1.0_V);
  } else {
    m_intake->setVoltage(0.0_V);
  }

}


// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {
  m_intake->setVoltage(0_V);
}


// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  return false;
}
