// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SupportArms/SupportArms.h"


class SupportArmsMoveByDistance
	: public frc2::CommandHelper<frc2::Command, SupportArmsMoveByDistance> {
public:
	struct Profile {
		SupportArmsState startingState;
		SupportArmsState targetState;
		units::meter_t profileActivationDistance;
	};

	SupportArmsMoveByDistance(SupportArms* supportArms, Profile profile, std::function<units::meter_t()> distanceToTargetProvider);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

private:
	SupportArms* supportArms;

	std::function<units::meter_t()> distanceToTargetProvider;
	units::meter_t distanceToTarget;

	double lowerAngleTravel = 0;

	Profile profile;
};
