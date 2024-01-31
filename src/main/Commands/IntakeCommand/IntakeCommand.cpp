// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeCommand.h"

#include "main/Commands/IntakeCommand/IntakeCommand.h"

IntakeCommand::IntakeCommand(Intake* m_Intake, units::volt_t m_voltage) {
  // Use addRequirements() here to declare subsystem dependencies.
    this->m_Intake = m_Intake;
    this->m_voltage = m_voltage;
    AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {
    m_Intake->setVoltage(m_voltage);
}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {}


// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {}


// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  return true;
}
