#pragma once

#include <frc2/command/Commands.h>
#include "main/Subsystems/Chassis/Chassis.h"
#include <photon/PhotonCamera.h>

frc2::CommandPtr AlignToTrackedObject(Chassis* chassis, photon::PhotonCamera* camera);