#include "SpeakerCommand.h"

frc2::CommandPtr SpeakerCommand(SuperStructure* superStructure, Shooter* shooter) {

	return frc2::cmd::Parallel(
		superStructure->superStructureCommand(SuperStructureConstants::ManualSpeakerState),
		shooter->shooterCommand(ShooterConstants::ManualSpeakerSpeed)
	);
}
