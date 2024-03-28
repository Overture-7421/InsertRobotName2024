// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceGrabCommand.h"


frc2::CommandPtr SourceGrabCommand(SuperStructure* superStructure, Shooter* shooter, Storage* storage) {
	return frc2::cmd::Sequence(
		SuperStructureCommand(superStructure, SuperStructureConstants::SourceGrabState).ToPtr(),
		ShooterCommand(shooter, ShooterConstants::SourceGrabSpeed).ToPtr(),
		StorageCommand(storage, StorageConstants::SourceGrabVolts).Repeatedly().Until(
			[=]() {
        		return storage->isNoteOnBackSensor();
      		}
		).AndThen(
			[=]() {
				shooter->setTargetVelocity(0.0);
				storage->setVoltage(0.0_V);
			}
		)
	);
}




