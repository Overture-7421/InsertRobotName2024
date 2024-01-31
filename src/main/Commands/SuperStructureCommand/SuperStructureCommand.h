#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "main/Subsystems/SuperStructure/SuperStructure.h"

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
  SuperStructure* m_SuperStructure;

  // State
  SuperStructureState m_targetState;
};