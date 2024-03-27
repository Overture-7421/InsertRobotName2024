// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceAutoCenterRace.h"

#include "main/Commands/SpeakerCommand/SpeakerCommand.h"
#include "main/Commands/VisionAmpCommand/VisionAmpCommand.h"
#include "main/Commands/VisionSourceGrabCommand/VisionSourceGrabCommand.h"
#include "main/Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"
#include "main/Commands/VisionSpeakerCommandNoShoot/VisionSpeakerCommandNoShoot.h"

frc2::CommandPtr SourceAutoCenterRace(Chassis* chassis, Storage* storage, Shooter* shooter) {

    return frc2::cmd::Sequence(
		pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAuto1")),
			pathplanner::NamedCommands::getCommand("GroundGrabCommand")
		),
		//Go back to shoot or grab next note if stolen
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Deadline(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAuto2")),
					pathplanner::NamedCommands::getCommand("VisionNoShoot")
				),
				pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAuto3")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAutoStolen1")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			[&] {return storage->isNoteOnForwardSensor();}
		),
		//Go back to shoot or grab next note if stolen
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Deadline(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAuto4")),
					pathplanner::NamedCommands::getCommand("VisionNoShoot")
				),
				pathplanner::NamedCommands::getCommand("VisionSpeakerCommand"),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAuto5")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAutoStolen2")),
					pathplanner::NamedCommands::getCommand("GroundGrabCommand")
				)
			),
			[&] {return storage->isNoteOnForwardSensor();}
		),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("SourceAuto6")),
			pathplanner::NamedCommands::getCommand("VisionNoShoot")
		),
		pathplanner::NamedCommands::getCommand("VisionSpeakerCommand")
	);
}
