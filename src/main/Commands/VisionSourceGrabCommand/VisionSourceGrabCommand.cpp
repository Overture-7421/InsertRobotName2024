// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSourceGrabCommand.h"
#include <frc2/command/ParallelCommandGroup.h>
#include "main/Commands/SourceGrabCommand/SourceGrabCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

frc2::CommandPtr VisionSourceGrabCommand(SuperStructure* superStucture, Shooter* shooter) {

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		3.0_mps, 3.0_mps_sq,
		560_deg_per_s, 720_deg_per_s_sq);

	return frc2::cmd::Sequence(
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::pathfindToPose(flipPoseIfNeeded({ 15.68_m, 0.60_m, {120_deg} }), constraints),
			SuperStructureCommand(superStucture, { 70.0, -50.0 }).ToPtr()
		),
		SourceGrabCommand(superStucture, shooter).ToPtr()
	);



}