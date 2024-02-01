#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "main/Commands/SuperStructureCommand/SuperStructureCommand.h"
#include "main/Commands/ShooterCommand/ShooterCommand.h"
#include "main/Commands/StorageCommand/StorageCommand.h"

class AmpCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AmpCommand> {
 public:
  AmpCommand(SuperStructure* superStucture, Shooter* shooter, Storage* storage);
};
