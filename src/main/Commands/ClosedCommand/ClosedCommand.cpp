// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include <frc2/command/ParallelDeadlineGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ClosedCommand::ClosedCommand(SuperStructure* superStructure, Intake* intake, Storage* storage, Shooter* shooter) {
  AddCommands(
    
    frc2::ParallelDeadlineGroup{
      StorageCommand(storage, 0_V),
      IntakeCommand(intake, 0_V),
      ShooterCommand(shooter, 0.00)
    },
    SuperStructureCommand(superStructure, SuperStructureConstants::GroundGrabState)

    );
}
