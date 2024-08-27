#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
	const static double LowerAngleGearRatio = 172.8;
	const static double UpperAngleGearRatio = 144.0;

	const static units::degree_t LowerAngleLowerLimit = -31.5_deg;
	const static units::degree_t LowerAngleUpperLimit = 90_deg;

	const static units::degree_t UpperAngleLowerLimit = -40_deg;
	const static units::degree_t UpperAngleUpperLimit = 90_deg;

	const static units::degree_t LowerAngleSafetyThreshold = -30_deg;
	const static units::degree_t UpperAngleSafetyLimit = 67_deg;

	const static SuperStructureState GroundGrabState = { LowerAngleLowerLimit, 68.0_deg };
	const static SuperStructureState NearShoot = { -31.5_deg, 68.0_deg };
	const static SuperStructureState ClosedState = { LowerAngleLowerLimit, UpperAngleUpperLimit };
	const static SuperStructureState ClimbEndState = { 0_deg, UpperAngleUpperLimit };
	const static SuperStructureState SourceGrabState = { LowerAngleLowerLimit, 89.65_deg };
	const static SuperStructureState ManualSpeakerState = { -12.0_deg, 55.0_deg };
	const static SuperStructureState AmpState = { 58.0_deg,  63.0_deg };
	const static SuperStructureState HighPassingState = { -12.0_deg, 51.0_deg };
	const static SuperStructureState LowPassingState = { 7.0_deg, 85.0_deg };
};