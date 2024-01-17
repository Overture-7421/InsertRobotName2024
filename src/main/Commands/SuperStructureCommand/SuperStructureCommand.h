#pragma once 

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "main/subsystems/SuperStructure/SuperStructure.h"
#include "main/subsystems/Intake/Intake.h"
#include "main/subsystems/SupportArms/SupportArms.h"

frc2::CommandPtr StartIntake(Intake* m_Intake);
frc2::CommandPtr StopIntake(Intake* m_Intake);
frc2::CommandPtr MoveStructure(SuperStructure* m_SuperStructure);
frc2::CommandPtr MoveArms(SuperStructure* m_SuperStructure);
