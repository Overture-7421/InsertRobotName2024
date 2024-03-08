#pragma once
#include <units/voltage.h>

namespace StorageConstants {
	const static units::volt_t GroundGrabVolts = 3.5_V;
	const static units::volt_t SpitVolts = -6_V;
	const static units::volt_t SourceGrabVolts = -3_V;
	const static units::volt_t AmpScoreVolts = 3_V;
	const static units::volt_t SpeakerScoreVolts = 5_V;
};