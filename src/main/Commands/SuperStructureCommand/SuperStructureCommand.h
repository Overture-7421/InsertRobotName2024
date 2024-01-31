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


class SuperStructureCommand
  : public frc2::CommandHelper<frc2::Command, SuperStructureCommand> {
public:
  SuperStructureCommand(SuperStructure* superStructure, SuperStructureState targetState);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  // Subsystem
  SuperStructure* superStructure;

  // State
  SuperStructureState targetState;
};