#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/XboxController.h>

#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/WaitForCheckPoint/WaitForCheckPoint.h"
#include "Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "Enums/StageLocation.h"


#include <vector>
#include <utility>

frc2::CommandPtr AutoClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, frc::XboxController* controller);
frc2::CommandPtr ManualClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, frc::XboxController* controller);

