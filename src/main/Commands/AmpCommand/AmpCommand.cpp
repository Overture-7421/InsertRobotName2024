#include "AmpCommand.h"

#include <frc2/command/ParallelDeadlineGroup.h>


frc2::CommandPtr AmpCommand(SuperStructure* superStucture, Shooter* shooter) {
	return frc2::cmd::Deadline(
		superStucture->superStructureCommand(SuperStructureConstants::AmpState),
		shooter->shooterCommand(ShooterConstants::AmpScoreSpeed)
	);
}
