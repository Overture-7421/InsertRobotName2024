#pragma once
#include <units/voltage.h>

namespace StorageConstants {
	const static units::volt_t GroundGrabVolts = 2.5_V;
	const static units::volt_t SpitVolts = -6_V;
	const static units::volt_t SourceGrabVolts = -3_V;
	const static units::volt_t AmpScoreVolts = 3_V;
	const static units::volt_t SpeakerScoreVolts = 12_V;
	// const static double IRActivationThreshold = 20;
	const static units::millimeter_t DistanceSensorActivationThreshold = 15_cm;
	const static units::second_t DistanceSensorAvailableTimeTolerance = 1_s;
};