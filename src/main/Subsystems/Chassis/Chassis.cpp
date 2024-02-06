// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Chassis.h"

Chassis::Chassis() {
	pigeon = &chassisPigeon;
	setModulePositions(&modulePos);
	setModules(&frontLeft, &frontRight, &backLeft, &backRight);
	setModulesRatios(turnRatio, driveRatio, wheelDiameter);
	setRotatorPID(53, 0, 0);

#ifndef __FRC_ROBORIO__
	frontLeft.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
	frontRight.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
	backLeft.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
	backRight.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
#else
	frontLeft.setFFConstants(0.15117_V, 1.9912_V, 0.032941_V);
	frontRight.setFFConstants(0.11106_V, 2.0075_V, 0.08823_V);
	backLeft.setFFConstants(0.030111_V, 2.0732_V, 0.16158_V);
	backRight.setFFConstants(0.09324_V, 2.0492_V, 0.077588_V);
#endif

}
