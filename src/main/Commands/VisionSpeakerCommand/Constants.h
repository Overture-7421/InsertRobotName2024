#pragma once

#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include <OvertureLib/Math/InterpolatingTable/InterpolatingTable.h>

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToLowerAngleTable{
		{1.66_m, -12.0_deg},
		{1.9_m, -12.0_deg},
		{2.4_m, -12.0_deg},
		{2.9_m, -12.0_deg},
		{3.4_m, -12.0_deg},
		{2.4_m, -12.0_deg},
		{3.9_m, -12.0_deg},
		{4.4_m, -13.0_deg},
		{4.9_m, -13.0_deg},
		{5.4_m, -13.0_deg},
		{5.9_m, -13.0_deg},
		{6.4_m, -13.0_deg},
		{6.9_m, -13.5_deg}

	};

	// Red Alliance
	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToUpperAngleTable{
		{1.66_m, 63.0_deg},
		{1.9_m, 67.0_deg},
		{2.4_m, 72.0_deg},
		{2.9_m, 75.0_deg},
		{3.4_m, 79.0_deg},
		{3.9_m, 82.0_deg},
		{4.4_m, 85.3_deg},
		{4.9_m, 86.2_deg},
		{5.4_m, 87.5_deg},
		{5.9_m, 88.0_deg},
		{6.4_m, 88.2_deg},
		{6.9_m, 88.7_deg}
	};

	// Blue Alliance
	// static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
	// 	{1.4_m, -35.0 + 1.75},
	// 	{1.9_m, -25.0 + 1.75},
	// 	{2.4_m, -18.5 + 1.75},
	// 	{2.9_m, -15.5 + 1.75},
	// 	{3.4_m, -12.5 + 1.5},
	// 	{3.9_m, -10.5 + 1.5},
	// 	{4.4_m, -9.0 + 1.5},
	// 	{4.9_m, -8.2 + 1.5},
	// 	{5.4_m, -7.2 + 1.5},
	// 	{5.9_m, -6.5 + 1.5},
	// 	{6.4_m, -6.0 + 1.5}
	// };

	static const InterpolatingTable<units::meter_t, double> DistanceToVelocityTable{
		{1.66_m, 100.0},
		{1.9_m, 100.0},
		{2.4_m, 110.0},
		{2.9_m, 130.0},
		{3.4_m, 140.0},
		{3.9_m, 140.0},
		{4.4_m, 140.0},
		{4.9_m, 140.0},
		{5.4_m, 150.0},
		{5.9_m, 150.0},
		{6.4_m, 150.0},
		{6.9_m, 165.0}


	};

	static const double multiplier = 1;

	static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1.66_m, 0.22_s * multiplier},
	  {1.9_m, 0.22_s * multiplier},
	  {2.4_m, 0.25_s * multiplier},
	  {2.9_m, 0.23_s * multiplier},
	  {3.4_m, 0.26_s * multiplier},
	  {3.9_m, 0.28_s * multiplier},
	  {4.4_m, 0.34_s * multiplier},
	  {4.9_m, 0.37_s * multiplier},
	  {5.4_m, 0.38_s * multiplier},
	  {5.9_m, 0.43_s * multiplier},
	  {6.4_m, 0.48_s * multiplier},
	  {6.9_m, 0.51_s * multiplier}


	};
};