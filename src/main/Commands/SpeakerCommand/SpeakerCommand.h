#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Shooter/Shooter.h"

frc2::CommandPtr SpeakerCommand(SuperStructure* superStructure, Shooter* shooter);

