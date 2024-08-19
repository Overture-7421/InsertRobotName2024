#pragma once

#include <units/voltage.h>

namespace IntakeConstants {
	const static units::volt_t GroundGrabVolts = -6_V;
	const static units::volt_t StopVolts = 0_V;
	const static units::volt_t ReverseVolts = 6_V;
};