// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <wpi/DataLog.h>

#include <frc/Timer.h>

#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include "OvertureLib/Math/Utils.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Shooter/Shooter.h"
#include "main/Subsystems/Storage/Storage.h"
#include "main/Subsystems/Targeting/TargetProvider.h"

#include "main/Commands/UtilityFunctions/UtilityFunctions.h"
#include "main/Commands/StorageCommand/StorageCommand.h"
#include "Constants.h"

class VisionSpeakerCommand
	: public frc2::CommandHelper<frc2::Command, VisionSpeakerCommand> {
public:
	VisionSpeakerCommand(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, TargetProvider* targetProvider, frc::XboxController* joystick);
	VisionSpeakerCommand(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, TargetProvider* targetProvider, Storage* storage);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;


	static void SetUpperAngleOffset(double offset);
	static double GetUpperAngleOffset();
	static void ResetUpperAngleOffset();

private:
	frc::Timer Timer;

	SuperStructure* superStructure;
	Chassis* chassis;
	Shooter* shooter;
	frc::XboxController* joystick = nullptr;
	Storage* storage;
	TargetProvider* targetProvider;

	bool lowerAngleInTolerance;
	bool upperAngleInTolerance;
	bool headingInTolerance;
	bool shooterSpeedInTolerance;

	TargetingWhileMoving dynamicTarget{
		VisionSpeakerCommandConstants::DistanceToShotTimeTable
	};

	units::meter_t distance = 0.0_m;
	frc::Rotation2d angle;

	static double UPPER_ANGLE_OFFSET;
	
	wpi::log::DataLog& log = frc::DataLogManager::GetLog();
	wpi::log::DoubleLogEntry upperAngleOffsetLog = wpi::log::DoubleLogEntry(log, "/vision_speaker_command/upper_angle_offset");
};
