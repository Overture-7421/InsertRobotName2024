// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include <frc/Timer.h>

#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include "OvertureLib/Math/Utils.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Shooter/Shooter.h"
#include "main/Subsystems/Storage/Storage.h"
#include "main/Subsystems/Vision/AprilTagCamera.h"

#include "main/Commands/UtilityFunctions/UtilityFunctions.h"
#include "main/Commands/StorageCommand/StorageCommand.h"

#include "main/Enums/PassNote.h"

class VisionSpeakerCommandPassNote
	: public frc2::CommandHelper<frc2::Command, VisionSpeakerCommandPassNote> {
public:
	VisionSpeakerCommandPassNote(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, AprilTagCamera* tagCamera, Storage* storage, PassNote upOrDown);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

private:
	frc::Timer Timer;

	SuperStructure* superStructure;
	Chassis* chassis;
	Shooter* shooter;
	Storage* storage;

	AprilTagCamera* tagCamera;

	bool lowerAngleInTolerance;
	bool upperAngleInTolerance;
	bool headingInTolerance;
	bool shooterSpeedInTolerance;

	PassNote upOrDown;
	SuperStructureState targetState;
	double targetShooterVelocity;

	units::meter_t distance = 0.0_m;
	frc::Rotation2d angle;
	frc::Translation2d speakerLoc;
};
