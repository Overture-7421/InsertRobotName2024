// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/InstantCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ClosedCommand::ClosedCommand(SuperStructure* superStructure, Intake* intake, Storage* storage, Shooter* shooter) {
  AddCommands(
    frc2::InstantCommand([=] {
      storage->setVoltage(0_V);
      intake->setVoltage(0_V);
      // shooter->setVelocityVoltage(50);
      superStructure->setTargetCoord(SuperStructureConstants::GroundGrabState);
    })
    );
}
