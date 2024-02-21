#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/XboxController.h>

#include "main/Subsystems/Chassis/Chassis.h"
#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/SupportArms/SupportArms.h"
#include "main/Subsystems/Vision/AprilTagCamera.h"
#include "main/Commands/UtilityFunctions/UtilityFunctions.h"
#include "main/Commands/WaitForCheckPoint/WaitForCheckPoint.h"
#include "main/Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "main/Commands/SupportArmCommand/SupportArmCommand.h"
#include "main/Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "main/Commands/SupportArmsMoveByDistance/SupportArmsMoveByDistance.h"
#include "main/Enums/StageLocation.h"


#include <vector>
#include <utility>

frc2::CommandPtr AutoClimb(Chassis* chassis, SuperStructure* superStructure, frc::XboxController* controller);

frc2::CommandPtr ManualClimb(Chassis* chassis, SuperStructure* superStructure, AprilTagCamera* aprilTagCamera, frc::XboxController* controller);

