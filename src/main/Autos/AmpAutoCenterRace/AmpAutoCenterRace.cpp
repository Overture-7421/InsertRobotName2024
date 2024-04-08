// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AmpAutoCenterRace.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

frc2::CommandPtr AmpAutoCenterRace(Storage* storage) {
    return frc2::cmd::Sequence(
		pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto1")),
			pathplanner::NamedCommands::getCommand("GroundGrabCommandLT")
		),
		//Go back to shoot or grab next note if stolen
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Deadline(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto2")),
					pathplanner::NamedCommands::getCommand("VisionNoShoot")
				),
				pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto3")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommandLT")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("RaceAmpAuto_Stolen1")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommandLT")
				)
			),
			[=] {return storage->isNoteOnForwardSensor();}
		),
		//Go back to shoot or grab next note if stolen
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Deadline(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto4")),
					pathplanner::NamedCommands::getCommand("VisionNoShoot")
				),
				pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto5")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommandLT")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("RaceAmpAuto_Stolen2")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommandLT")
				)
			),
			[=] {return storage->isNoteOnForwardSensor();}
		),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AMPAuto6")),
			pathplanner::NamedCommands::getCommand("VisionNoShoot")
		),
		pathplanner::NamedCommands::getCommand("GroundGrabCommandNT")
	);
}


