#pragma once
#include <units/voltage.h>

namespace StorageConstants {
	const static units::volt_t GroundGrabVolts = 5_V;
	const static units::volt_t SpitVolts = -5_V;
	const static units::volt_t ScoreVolts = 12_V;
	const static units::volt_t StopVolts = 0_V;
	// const static double IRActivationThreshold = 20;
	const static units::millimeter_t DistanceSensorActivationThreshold = 15_cm;
	const static units::second_t DistanceSensorAvailableTimeTolerance = 1_s;
};