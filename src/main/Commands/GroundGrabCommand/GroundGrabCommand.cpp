// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GroundGrabCommand.h"


frc2::CommandPtr GroundGrabCommand(SuperStructure* superStructure, Storage* storage, Intake* intake) {

  return frc2::cmd::Sequence(
    SuperStructureCommand (superStructure, SuperStructureConstants::GroundGrabState).ToPtr(), 
    frc2::cmd::Parallel(
        IntakeCommand(intake, IntakeConstants::GroundGrabVolts).ToPtr(),
        StorageCommand(storage, StorageConstants::GroundGrabVolts).ToPtr()
    ).Repeatedly().Until(
      [=]() {
        return storage->isNoteOnForwardSensor();
      }
    ).AndThen(
      frc2::cmd::Wait(0.0_s) //0.015
    ).AndThen(
      [=]() {
        storage->setVoltage(0.0_V);
        intake->setVoltage(0.0_V);
      }
    )
  );
}