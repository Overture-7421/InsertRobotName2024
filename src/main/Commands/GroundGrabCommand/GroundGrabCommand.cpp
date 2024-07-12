// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GroundGrabCommand.h"


frc2::CommandPtr GroundGrabCommand(SuperStructure* superStructure, Storage* storage, Intake* intake, bool ignoreSensor) {

	if (ignoreSensor) {
		return frc2::cmd::Sequence(
			superStructure->superStructureCommand(SuperStructureConstants::GroundGrabState),
			frc2::cmd::Parallel(
				intake->intakeCommand(IntakeConstants::GroundGrabVolts),
				storage->storageCommand(StorageConstants::GroundGrabVolts)
			)
		);
	}


	return frc2::cmd::Sequence(
		superStructure->superStructureCommand(SuperStructureConstants::GroundGrabState),
		frc2::cmd::Parallel(
			intake->intakeCommand(IntakeConstants::GroundGrabVolts),
			storage->storageCommand(StorageConstants::GroundGrabVolts)
		).Repeatedly().Until(
			[=]() {
		return storage->isNoteOnForwardSensor();
	}
		)
		// .AndThen(
		// 	frc2::cmd::WaitUntil(0.02_s)
		// )
		.FinallyDo(
			[=]() {
		storage->storageCommand(StorageConstants::StopVolts);
		intake->intakeCommand(IntakeConstants::StopVolts);
	}
		)
	);
}