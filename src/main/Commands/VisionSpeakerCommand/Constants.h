#pragma once

#include <units/length.h>
#include <units/time.h>
#include <frc/geometry/Translation2d.h>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

namespace VisionSpeakerCommandConstants {
	static const InterpolatingTable<units::meter_t, double> DistanceToLowerAngleTable{
		{1.66_m, -12.0},
		{1.9_m, -12.0},
		{2.4_m, -12.0}, 
		{2.9_m, -12.0},
		{3.4_m, -12.0},
		{2.4_m, -12.0},
		{3.9_m, -12.0},
		{4.4_m, -12.0},
		{4.9_m, -12.0},
		{5.4_m, -12.0},
		{5.9_m, -12.0}
	};

	// Red Alliance
	static const InterpolatingTable<units::meter_t, double> DistanceToUpperAngleTable{
		{1.66_m, -29.0},
		{1.9_m, -27.0},
		{2.4_m, -19.0},
		{2.9_m, -14.5},
		{3.4_m, -10.75},
		{3.9_m, -8.0},
		{4.4_m, -7.0},
		{4.9_m, -4.5},
		{5.4_m, -3.0},
		{5.9_m, -2.0}
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
		{5.4_m, 140.0},
		{5.9_m, 140.0}
	};

	static const InterpolatingTable<units::meter_t, units::second_t> DistanceToShotTimeTable{
	  {1.66_m, 0.15_s},
	  {1.9_m, 0.155_s},
	  {2.4_m, 0.16_s}, 
	  {2.9_m, 0.19_s}, 
	  {3.4_m, 0.20_s}, 
	  {3.9_m, 0.24_s},
	  {4.4_m, 0.25_s},
	  {4.9_m, 0.28_s},
	  {5.4_m, 0.35_s},
	  {5.9_m, 0.36_s}
	};
};