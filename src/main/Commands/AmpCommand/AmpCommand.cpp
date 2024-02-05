#include "AmpCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AmpCommand::AmpCommand(SuperStructure* superStucture, Shooter* shooter, Storage* storage) {

	AddCommands(
		frc2::ParallelCommandGroup(
			SuperStructureCommand(superStucture, { 65.0, -20.0 }),
			ShooterCommand(shooter, 6.00)
		)
	);
}
