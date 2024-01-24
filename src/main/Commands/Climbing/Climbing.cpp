#include "Climbing.h"

frc2::CommandPtr Climb() {
	auto path = pathplanner::PathPlannerPath::fromPathFile("AMP Climb");

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		1.0_mps, 1.0_mps_sq,
		180_deg_per_s, 180_deg_per_s_sq);

	return pathplanner::AutoBuilder::pathfindThenFollowPath(
		path,
		constraints
	);
};
