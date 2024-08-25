// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TabulateCommand.h"

TabulateCommand::TabulateCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider) : headingHelper({ 11.0, 0.5, 0.35, {18_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->chassis = chassis;
	this->targetProvider = targetProvider;

	AddRequirements({ superStructure, shooter });
}

// Called when the command is initially scheduled.
void TabulateCommand::Initialize() {
	frc::SmartDashboard::PutNumber("Tabulate/LowerAngle", superStructure->getLowerAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/UpperAngle", superStructure->getUpperAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVel", 0.0);

	targetLocation = targetProvider->GetSpeakerLocation();

	chassis->enableSpeedHelper(&headingHelper);
}

// Called repeatedly when this Command is scheduled to run
void TabulateCommand::Execute() {

	frc::Pose2d chassisPose = chassis->getEstimatedPose();
	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = targetLocation - chassisLoc;
	auto distance = chassisToTarget.Distance({ 0_m, 0_m });
	auto angle = chassisToTarget.Angle().RotateBy({ 180_deg });

	frc::SmartDashboard::PutNumber("Tabulate/Distance", distance.value());
	frc::SmartDashboard::PutNumber("Tabulate/LowerAngleCurrent", superStructure->getLowerAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/UpperAngleCurrent", superStructure->getUpperAngle().value());
	frc::SmartDashboard::PutNumber("Tabulate/ShooterVelCurrent", shooter->getCurrentVelocity());

	headingHelper.setTargetAngle(angle.Radians());
	units::degree_t lowerAngle{ frc::SmartDashboard::GetNumber("Tabulate/LowerAngle", superStructure->getLowerAngle().value()) };
	units::degree_t upperAngle{ frc::SmartDashboard::GetNumber("Tabulate/UpperAngle", superStructure->getLowerAngle().value()) };
	double targetVel = frc::SmartDashboard::GetNumber("Tabulate/ShooterVel", 0.0);

	superStructure->setTargetCoord({ lowerAngle, upperAngle });
	shooter->setTargetVelocity(targetVel);
}

// Called once the command ends or is interrupted.
void TabulateCommand::End(bool interrupted) {
	shooter->setTargetVelocity(0.0);
	chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool TabulateCommand::IsFinished() {
	return false;
}
