#pragma once 

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "main/Subsystems/SuperStructure/SuperStructure.h"
#include "main/Subsystems/Intake/Intake.h"
#include "main/Subsystems/SupportArms/SupportArms.h"
#include "main/Subsystems/Storage/Storage.h"
#include "main/Subsystems/Shooter/Shooter.h"


frc2::CommandPtr StartIntake(Intake* m_Intake, SuperStructure* m_SuperStructure, Storage* m_Storage);
frc2::CommandPtr StopIntake(Intake* m_Intake, SuperStructure* m_SuperStructure, Storage* m_Storage);
frc2::CommandPtr ShootingPose(Intake* m_Intake, SuperStructure* m_SuperStructure);
frc2::CommandPtr AngleShootingProcess(double* angleShooting);