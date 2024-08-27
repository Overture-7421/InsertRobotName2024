// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowShoot.h"
#include "Commands/LowShoot/LowShoot.h"
#include <frc/MathUtil.h>

LowShoot::LowShoot(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider, Storage* storage) : headingHelper({ 11.0, 0.5, 0.6, {13_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->targetProvider = targetProvider;
	this->storage = storage;
}

LowShoot::LowShoot(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider) : headingHelper({ 11.0, 0.5, 0.6, {13_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->targetProvider = targetProvider;
	this->storage = nullptr;
}

// Called when the command is initially scheduled.
void LowShoot::Initialize() {

	targetLocation = targetProvider->GetSpeakerLocation();

	if (storage != nullptr) {
		storage->setVoltage(0_V);
	}

	chassis->enableSpeedHelper(&headingHelper);

	Timer.Reset();
	Timer.Stop();
}

// Called repeatedly when this Command is scheduled to run
void LowShoot::Execute() {
	frc::Pose2d chassisPose = chassis->getEstimatedPose();
	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = targetLocation - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ -180_deg });


	headingHelper.setTargetAngle(angle.Radians());
	units::degree_t targetLowerAngle = LowShootConstants::DistanceToLowerAngleTable[distance];
	units::degree_t targetUpperAngle = LowShootConstants::DistanceToUpperAngleTable[distance];
	double targetShooterVelocity = LowShootConstants::DistanceToVelocityTable[distance];
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setTargetVelocity(targetShooterVelocity);


	units::degree_t headingTolerance = 2_deg + units::degree_t(std::clamp(1 - distance.value() / 6.0, 0.0, 1.0) * 8.0); // Heading tolerance extra of X deg when close, more precise when further back;
	units::degree_t headingError = units::math::abs(frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg));

	bool lowerAngleInTolerance = std::abs((targetLowerAngle - superStructure->getLowerAngle()).value()) < 1;

	bool upperAngleInTolerance = std::abs((targetUpperAngle - superStructure->getUpperAngle()).value()) < 1;

	bool headingInTolerance = headingError < headingTolerance;
	bool shooterSpeedInTolerance = (targetShooterVelocity - 5.0) < shooter->getCurrentVelocity();

	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/LowerAngleReached", lowerAngleInTolerance);
	frc::SmartDashboard::PutNumber("VisionSpeakerCommand/Distance", distance.value());
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/UpperAngleReached", upperAngleInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/HeadingReached", headingInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/ShooterReached", shooterSpeedInTolerance);
	if (lowerAngleInTolerance && upperAngleInTolerance && headingInTolerance && shooterSpeedInTolerance) {
		if (storage != nullptr) {
			Timer.Start();
			storage->setVoltage(StorageConstants::ScoreVolts);
		} else {
			End(false);
		}
	}
}

// Called once the command ends or is interrupted.
void LowShoot::End(bool interrupted) {
	chassis->disableSpeedHelper();

	if (storage != nullptr) {
		storage->setVoltage(0_V);
	}
}

// Returns true when the command should end.
bool LowShoot::IsFinished() {

	if (storage != nullptr) {
		if (Timer.Get() > 0.4_s) {
			return true;
		}
	}


	return false;
}