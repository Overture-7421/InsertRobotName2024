// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Chassis.h"

Chassis::Chassis() : SwerveChassis(ChassisConstants::MaxModuleSpeed, ChassisConstants::DriveBaseRadius) {
	pigeon = &chassisPigeon;
	setModulePositions(&modulePos);
	setModules(&frontLeft, &frontRight, &backLeft, &backRight);
	setModulesRatios(ChassisConstants::RotationGearRatio, ChassisConstants::DriveGearRatio, ChassisConstants::WheelDiameter.value());
	setRotatorPID(53, 0, 0);

#ifndef __FRC_ROBORIO__
	frontLeft.setFFConstants(0.040347_V, 1.93_V, 0.63363_V);
	frontRight.setFFConstants(0.26831_V, 1.9683_V, 0.1204_V);
	backLeft.setFFConstants(0.0089267_V, 1.8401_V, 0.77189_V);
	backRight.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
#else
	frontLeft.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
	frontRight.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
	backLeft.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
	backRight.setFFConstants(0.22436_V, 2.0254_V, 0.2019_V);
#endif

}
