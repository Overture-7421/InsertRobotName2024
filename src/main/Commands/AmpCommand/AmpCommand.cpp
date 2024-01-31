#include "AmpCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AmpCommand::AmpCommand(SuperStructure* superStucture) {

  AddCommands(
    SuperStructureCommand(superStucture, {50.0, 30.0})
  );
}
