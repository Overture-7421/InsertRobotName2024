// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceGrabCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SourceGrabCommand::SourceGrabCommand(SuperStructure* superStructure, Storage* storage) {

  AddCommands(
    SuperStructureCommand(superStructure, {30.0, -60.0}),
    StorageCommand(storage, 3_V)
  );
}
