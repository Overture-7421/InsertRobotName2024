#include "AmpCommand.h"

#include <frc2/command/ParallelDeadlineGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AmpCommand::AmpCommand(SuperStructure* superStucture, Shooter* shooter) {

	AddCommands(
		frc2::ParallelDeadlineGroup(
			SuperStructureCommand(superStucture, SuperStructureConstants::AmpState),
			ShooterCommand(shooter, ShooterConstants::AmpScoreSpeed)
		)
	);
}
