#pragma once
#include "SuperStructureState.h"

namespace SuperStructureConstants {
	const static double LowerAngleGearRatio = 172.8;
	const static double UpperAngleGearRatio = 144.0;

	const static units::degree_t LowerAngleLowerLimit = -31.5_deg;
	const static units::degree_t LowerAngleUpperLimit = 90_deg;

	const static units::degree_t UpperAngleLowerLimit = -130_deg;
	const static units::degree_t UpperAngleUpperLimit = -0.2_deg;

	const static units::degree_t LowerAngleSafetyThreshold = -30_deg;
	const static units::degree_t UpperAngleSafetyLimit = -23_deg;

	const static SuperStructureState GroundGrabState = { LowerAngleLowerLimit, -22.0_deg };
	const static SuperStructureState NearShoot = { LowerAngleLowerLimit, -18.0_deg };
	const static SuperStructureState ClosedState = { LowerAngleLowerLimit, UpperAngleUpperLimit };
	const static SuperStructureState ClimbEndState = { 0_deg, UpperAngleUpperLimit };
	const static SuperStructureState SourceGrabState = { LowerAngleLowerLimit, -0.35_deg };
	const static SuperStructureState ManualSpeakerState = { -12.0_deg, -35.0_deg };
	const static SuperStructureState AmpState = { 58.0_deg,  -27.0_deg };
	const static SuperStructureState SmoothCloseState = { -26.0_deg, -7.0_deg };
	const static SuperStructureState HighPassingState = { -12.0_deg, -39.0_deg };
	const static SuperStructureState LowPassingState = { 7.0_deg, -5.0_deg };
};