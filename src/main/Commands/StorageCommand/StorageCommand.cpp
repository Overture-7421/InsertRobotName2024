// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StorageCommand.h"

#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>

StorageCommand::StorageCommand(Storage* m_Storage, units::volt_t m_voltage) {
  // Use addRequirements() here to declare subsystem dependencies.

  this->m_voltage = m_voltage;
  this->m_Storage = m_Storage;
  AddRequirements({ m_Storage });
}

// Called when the command is initially scheduled.
void StorageCommand::Initialize() {
  m_Storage->setVoltage(m_voltage);
}

// Called repeatedly when this Command is scheduled to run
void StorageCommand::Execute() {
}

// Called once the command ends or is interrupted.
void StorageCommand::End(bool interrupted) {
}

// Returns true when the command should end.
bool StorageCommand::IsFinished() {
  return true;
}
