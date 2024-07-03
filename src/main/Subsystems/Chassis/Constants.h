#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <frc/geometry/Translation2d.h>

namespace ChassisConstants {
	const static double RotationGearRatio = 150.0 / 7.0;
	const static double DriveGearRatio = 5.9027777;
	const static units::meters_per_second_t MaxModuleSpeed = 5.39_mps;
	const static units::radians_per_second_t MaxAngularSpeed = 1.5_tps;
	const static units::meter_t DriveBaseRadius = 0.3732276_m;
	const static units::meter_t WheelDiameter = 0.1016_m * 0.92875809000609675765430314087843;

	const static frc::Translation2d FrontLeftModuleTranslation = { 7.506890_in, 10.375_in };
	const static frc::Translation2d FrontRightModuleTranslation = { 7.506890_in, -10.375_in };
	const static frc::Translation2d BackLeftModuleTranslation = { -13.243110_in, 10.375_in };
	const static frc::Translation2d BackRightModuleTranslation = { -13.243110_in, -10.375_in };
};