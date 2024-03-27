// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include "main/Subsystems/Shooter/Shooter.h"
#include "main/Subsystems/Storage/Storage.h"
#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/Vision/AprilTagCamera.h"

frc2::CommandPtr SourceAutoCenterRace(Chassis* chassis, Storage* storage, Shooter* shooter);
