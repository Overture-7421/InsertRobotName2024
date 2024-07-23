// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/HeadingSpeedsHelper/HeadingSpeedsHelper.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Commands/VisionSpeakerCommand/Constants.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Subsystems/Targeting/TargetProvider.h"

class VisionSpeakerCommandNoShoot
	: public frc2::CommandHelper<frc2::Command, VisionSpeakerCommandNoShoot> {
public:
	VisionSpeakerCommandNoShoot(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, TargetProvider* targetProvider);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

private:
	SuperStructure* superStructure;
	Chassis* chassis;
	Shooter* shooter;
	TargetProvider* targetProvider;
	

	frc::Translation2d targetLocation;

	units::meter_t distance = 0.0_m;
	frc::Rotation2d angle;
	HeadingSpeedsHelper headingHelper;
};