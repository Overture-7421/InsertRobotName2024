#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <frc/geometry/Translation2d.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/ModuleConfig.h>

namespace ChassisConstants {
	const static double TurnGearRatio = 150.0 / 7.0;
	const static double DriveGearRatio = 5.9027777;
	const static units::meters_per_second_t MaxModuleSpeed = 5.39_mps;
	const static units::radians_per_second_t MaxAngularSpeed = 1.5_tps;
	const static units::meter_t DriveBaseRadius = 0.3732276_m;
	const static units::meter_t WheelDiameter = 0.1016_m * 0.92875809000609675765430314087843;

	const static frc::Translation2d FrontLeftModuleTranslation = { 7.625_in, 10.375_in };
	const static frc::Translation2d FrontRightModuleTranslation = { 7.625_in, -10.375_in };
	const static frc::Translation2d BackLeftModuleTranslation = { -13.125_in, 10.375_in };
	const static frc::Translation2d BackRightModuleTranslation = { -13.125_in, -10.375_in };

	const static std::string CanBus = "OverCANivore";

	static ModuleConfig GetFrontLeftModuleConfig() {
		static ModuleConfig config{
			{0.22436_V, 2.0254_V / 1_mps, 0.2019_V / 1_mps_sq}
		};

		config.DrivedId = 5;
		config.TurnId = 6;
		config.CanCoderId = 11;
		config.Offset = 0_deg;
		config.ModuleName = "FrontLeftModule";
		config.CanBus = ChassisConstants::CanBus;
		config.DriveNeutralMode = ControllerNeutralMode::Brake;
		config.TurnNeutralMode = ControllerNeutralMode::Coast;
		config.DriveGearRatio =  ChassisConstants::DriveGearRatio;
		config.TurnGearRatio =  ChassisConstants::TurnGearRatio;
		config.WheelDiameter =  ChassisConstants::WheelDiameter;
		config.kP = 53.0;

		return config;
	}

	static ModuleConfig GetFrontRightModuleConfig() {
		static ModuleConfig config{
			{0.22436_V, 2.0254_V / 1_mps, 0.2019_V / 1_mps_sq}
		};

		config.DrivedId = 7;
		config.TurnId = 8;
		config.CanCoderId = 12;
		config.Offset = 0_deg;
		config.ModuleName = "FrontRightModule";
		config.CanBus = ChassisConstants::CanBus;
		config.DriveNeutralMode = ControllerNeutralMode::Brake;
		config.TurnNeutralMode = ControllerNeutralMode::Coast;
		config.DriveGearRatio =  ChassisConstants::DriveGearRatio;
		config.TurnGearRatio =  ChassisConstants::TurnGearRatio;
		config.WheelDiameter =  ChassisConstants::WheelDiameter;
		config.kP = 53.0;

		return config;
	}

	static ModuleConfig GetBackLeftModuleConfig() {
		static ModuleConfig config{
			{0.22436_V, 2.0254_V / 1_mps, 0.2019_V / 1_mps_sq}
		};

		config.DrivedId = 3;
		config.TurnId = 4;
		config.CanCoderId = 10;
		config.Offset = 0_deg;
		config.ModuleName = "BackLeftModule";
		config.CanBus = ChassisConstants::CanBus;
		config.DriveNeutralMode = ControllerNeutralMode::Brake;
		config.TurnNeutralMode = ControllerNeutralMode::Coast;
		config.DriveGearRatio =  ChassisConstants::DriveGearRatio;
		config.TurnGearRatio =  ChassisConstants::TurnGearRatio;
		config.WheelDiameter =  ChassisConstants::WheelDiameter;
		config.kP = 53.0;

		return config;
	}

	static ModuleConfig GetBackRightModuleConfig() {
		static ModuleConfig config{
			{0.22436_V, 2.0254_V / 1_mps, 0.2019_V / 1_mps_sq}
		};

		config.DrivedId = 1;
		config.TurnId = 2;
		config.CanCoderId = 9;
		config.Offset = 0_deg;
		config.ModuleName = "BackRightModule";
		config.CanBus = ChassisConstants::CanBus;
		config.DriveNeutralMode = ControllerNeutralMode::Brake;
		config.TurnNeutralMode = ControllerNeutralMode::Coast;
		config.DriveGearRatio =  ChassisConstants::DriveGearRatio;
		config.TurnGearRatio =  ChassisConstants::TurnGearRatio;
		config.WheelDiameter =  ChassisConstants::WheelDiameter;
		config.kP = 53.0;

		return config;
	}
};