#pragma once

#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, double> DistanceToLowerAngleTable{
		{1.4_m, -15.0},
		{1.9_m, -15.0},
		{2.4_m, -15.0},
		{2.9_m, -15.0},
		{3.4_m, -15.0},
		{3.9_m, -15.0},
		{4.4_m, -15.0},
		{4.9_m, -15.0},
		{5.4_m, -15.0},
		{5.9_m, -15.0},
		{6.4_m, -15.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
		{1.4_m, -35.0},
		{1.9_m, -25.0},
		{2.4_m, -18.5},
		{2.9_m, -16.0},
		{3.4_m, -13.0},
		{3.9_m, -11.0},
		{4.4_m, -10.0},
		{4.9_m, -8.0},
		{5.4_m, -7.2},
		{5.9_m, -6.5},
		{6.4_m, -6.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
		{1.4_m, 170.0},
		{1.9_m, 170.0},
		{2.4_m, 170.0},
		{2.9_m, 170.0},
		{3.4_m, 170.0},
		{3.9_m, 170.0},
		{4.4_m, 170.0},
		{4.9_m, 170.0},
		{5.4_m, 170.0},
		{5.9_m, 170.0},
		{6.4_m, 170.0}
	};

	static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1.4_m, 0.18_s},
	  {1.9_m, 0.20_s},
	  {2.4_m, 0.21_s},
	  {2.9_m, 0.25_s},
	  {3.4_m, 0.30_s},
	  {3.9_m, 0.32_s},
	  {4.4_m, 0.37_s},
	  {4.9_m, 0.41_s},
	  {5.4_m, 0.43_s},
	  {5.9_m, 0.48_s},
	  {6.4_m, 0.51_s}
	};

	static const frc::Translation2d TargetLocation = { 0.06_m, 5.55_m };
};