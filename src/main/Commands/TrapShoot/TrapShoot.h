// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// No se si se usa

#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/XboxController.h>
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Storage/Storage.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Enums/StageLocation.h"

#include <vector>
#include <utility>

const std::vector<std::pair<StageLocation, frc::Pose2d>> StageLocations{
	{StageLocation::Left, {{4.52_m, 4.67_m}, {120_deg}}},
	{StageLocation::Right,{{4.57_m, 3.51_m}, {-120_deg}}},
	{StageLocation::Back, {{5.51_m, 4.10_m}, {0_deg}}}
};

frc2::CommandPtr TrapShoot(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Shooter* shooter, Storage* storage);