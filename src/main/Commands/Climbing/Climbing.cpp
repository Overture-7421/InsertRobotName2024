#include "Climbing.h"

frc2::CommandPtr Climb() {
	auto path = pathplanner::PathPlannerPath::fromPathFile("AMP Climb");

	pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
		4.0_mps, 4.0_mps_sq,
		180_deg_per_s, 180_deg_per_s_sq);

	return pathplanner::AutoBuilder::pathfindThenFollowPath(
		path,
		constraints
	);
};
