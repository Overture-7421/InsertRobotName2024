// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ClosedCommand::ClosedCommand(SuperStructure* superStructure, Intake* intake, Storage* storage, Shooter* shooter) {
  AddCommands(
    
    frc2::ParallelCommandGroup{
    ShooterCommand(shooter, 0.00),
    IntakeCommand(intake, 0_V),
    StorageCommand(storage, 0_V)
    },
    SuperStructureCommand(superStructure, SuperStructureConstants::GroundGrabState)

    );
}
