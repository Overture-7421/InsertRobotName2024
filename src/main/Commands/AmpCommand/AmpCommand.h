#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/SuperStructure/SuperStructure.h"

frc2::CommandPtr AmpCommand(SuperStructure* superStucture, Shooter* shooter);
