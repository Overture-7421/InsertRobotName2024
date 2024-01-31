#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/XboxController.h>

#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/SupportArms/SupportArms.h"
#include "main/Commands/UtilityFunctions/UtilityFunctions.h"
#include "main/Commands/WaitForCheckPoint/WaitForCheckPoint.h"
#include "main/Enums/StageLocation.h"


#include <vector>
#include <utility>

frc2::CommandPtr Climb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, frc::XboxController* controller);

