// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionAmpCommand.h"
#include <frc2/command/ParallelCommandGroup.h>
#include "main/Commands/AmpCommand/AmpCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

frc2::CommandPtr VisionAmpCommand(SuperStructure* superStucture, Shooter* shooter, Storage* storage){

  pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		2.0_mps, 1.0_mps_sq,
		560_deg_per_s, 720_deg_per_s_sq);

 return frc2::cmd::Sequence(
  pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded({1.88_m, 7.39_m, {-90_deg}}),constraints),
  AmpCommand(superStucture, shooter, storage).ToPtr()
 );
}