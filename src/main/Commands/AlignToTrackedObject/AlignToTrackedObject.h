#pragma once

#include <frc2/command/Commands.h>
#include <photon/PhotonCamera.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/angular_acceleration.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Helpers/AlignRobotRelativeHelper/AlignRobotRelativeHelper.h"
#include "Helpers/AlignFieldRelativeHelper/AlignFieldRelativeHelper.h"
#include "Constants.h"

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera, AlignRobotRelativeHelper* alignHelper);

frc2::CommandPtr AlignToTrackedObjectFieldOriented(Chassis* chassis, photon::PhotonCamera* camera, AlignFieldRelativeHelper* alignController);