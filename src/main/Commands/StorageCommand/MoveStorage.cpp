// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "MoveStorage.h"

#include <frc2/command/button/Trigger.h>
#include <frc/XboxController.h>

MoveStorage::MoveStorage(Storage* m_Storage, units::volt_t m_voltage) {
  // Use addRequirements() here to declare subsystem dependencies.

  this->m_voltage = m_voltage;
  this->m_Storage = m_Storage;
  AddRequirements({ m_Storage });
}

// Called when the command is initially scheduled.
void MoveStorage::Initialize() {

  m_Storage->setVoltage(m_voltage);

}

// Called repeatedly when this Command is scheduled to run
void MoveStorage::Execute() {
}

// Called once the command ends or is interrupted.
void MoveStorage::End(bool interrupted) {
  m_Storage->setVoltage(0_V);
}

// Returns true when the command should end.
bool MoveStorage::IsFinished() {
  return false;
}
