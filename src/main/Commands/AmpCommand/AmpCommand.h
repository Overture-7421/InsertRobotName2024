#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "Commands/ShooterCommand/ShooterCommand.h"

class AmpCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AmpCommand> {
 public:
  AmpCommand(SuperStructure* superStucture, Shooter* shooter);
};
