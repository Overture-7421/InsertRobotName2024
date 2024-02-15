// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include "OvertureLib/Math/Utils.h"
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Shooter/Shooter.h"

#include "main/Commands/UtilityFunctions/UtilityFunctions.h"

class VisionSpeakerCommand
    : public frc2::CommandHelper<frc2::Command, VisionSpeakerCommand> {
 public:
  VisionSpeakerCommand(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, frc::XboxController* joystick);
  VisionSpeakerCommand(Chassis* chassis, SuperStructure* SuperStructure, Shooter* shooter, Storage* storage);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
	frc::Timer Timer;

	InterpolatingTable<units::meter_t, double> distanceToLowerAngleTable{
		{1.4_m, -10.0},
		{1.9_m, -10.0},
		{2.4_m, -10.0},
		{2.9_m, -10.0},
		{3.4_m, -10.0},
		{3.9_m, -10.0}
	};

	InterpolatingTable<units::meter_t, double> distanceToUpperAngleTable{
		{1.4_m, -30.0},
		{1.9_m, -25.0},
		{2.4_m, -22.0},
		{2.9_m, -18.0},
		{3.4_m, -15.0},
		{3.9_m, -13.0}
	};

	InterpolatingTable<units::meter_t, double> distanceToVelocityTable{
		{1.4_m, 100.0},
		{1.9_m, 105.0},
		{2.4_m, 110.0},
		{2.9_m, 115.0},
		{3.4_m, 120.0},
		{3.9_m, 125.0}
	};

	SuperStructure* superStructure;
	Chassis* chassis;
	Shooter* shooter;
	frc::XboxController* joystick = nullptr;
	Storage* storage;

	TargetingWhileMoving dynamicTarget { 
		{
			{1.4_m, 0.40_s}, 
			{1.9_m, 0.40_s}, 
			{2.4_m, 0.40_s}, 
			{2.9_m, 0.40_s}, 
			{3.4_m, 0.40_s}, 
			{3.9_m, 0.40_s}, 
		}
	};
  units::meter_t distance = 0.0_m;
  frc::Rotation2d angle;
};