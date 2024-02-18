#pragma once

#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, double> DistanceToLowerAngleTable{
		{1.4_m, -12.0},
		{1.9_m, -12.0},
		{2.4_m, -12.0},
		{2.9_m, -12.0},
		{3.4_m, -12.0},
		{3.9_m, -12.0},
		{4.4_m, -12.0},
		{4.9_m, -12.0},
		{5.4_m, -12.0},
		{5.9_m, -12.0},
		{6.4_m, -12.0}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
		{1.4_m, -37.0},
		{1.9_m, -30.0},
		{2.4_m, -23.0},
		{2.9_m, -19.0},
		{3.4_m, -16.0},
		{3.9_m, -15.5},
		{4.4_m, -13.0},
		{4.9_m, -10.0},
		{5.4_m, -9.0},
		{5.9_m, -7.6},
		{6.4_m, -6.7}
	};

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
		{1.4_m, 100.0},
		{1.9_m, 105.0},
		{2.4_m, 110.0},
		{2.9_m, 115.0},
		{3.4_m, 120.0},
		{3.9_m, 125.0},
		{4.4_m, 130.0},
		{4.9_m, 135.0},
		{5.4_m, 140.0},
		{5.9_m, 145.0},
		{6.4_m, 150.0}
	};

	static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1.4_m, 0.15_s},
	  {1.9_m, 0.20_s},
	  {2.4_m, 0.20_s},
	  {2.9_m, 0.25_s},
	  {3.4_m, 0.25_s},
	  {3.9_m, 0.30_s},
	  {4.4_m, 0.30_s},
	  {4.9_m, 0.40_s},
	  {5.4_m, 0.40_s},
	  {5.9_m, 0.40_s},
	  {6.4_m, 0.50_s}
	};

	static const frc::Translation2d TargetLocation = { 0.06_m, 5.55_m };
};