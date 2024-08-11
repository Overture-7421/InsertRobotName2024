// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"

const double UPPER_ANGLE_BLUE_DEFAULT_OFFSET = 0.0;
const double UPPER_ANGLE_RED_DEFAULT_OFFSET = 0;

double UPPER_ANGLE_OFFSET = 0;

#include <frc/MathUtil.h>

VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider, frc::XboxController* joystick) : headingHelper({ 11.0, 0.5, 0.6, {13_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });

	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->joystick = joystick;
	this->targetProvider = targetProvider;
}

VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superStructure, Shooter* shooter, TargetProvider* targetProvider, Storage* storage) : headingHelper({ 18.0, 0.5, 0.0, {13_rad_per_s, 18_rad_per_s_sq * 2} }, chassis) {
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter, storage });
	this->chassis = chassis;
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->storage = storage;
	this->targetProvider = targetProvider;
}

// Called when the command is initially scheduled.
void VisionSpeakerCommand::Initialize() {
	chassis->enableSpeedHelper(&headingHelper);

	dynamicTarget.setTargetLocation(targetProvider->GetSpeakerLocation());

	if (storage != nullptr) {
		storage->setVoltage(0_V);
	}

	Timer.Reset();
	Timer.Stop();
}

void VisionSpeakerCommand::LoadAllianceOffset() {
	if (isRedAlliance()) {
		VisionSpeakerCommand::SetUpperAngleOffset(UPPER_ANGLE_RED_DEFAULT_OFFSET);
	} else {
		VisionSpeakerCommand::SetUpperAngleOffset(UPPER_ANGLE_BLUE_DEFAULT_OFFSET);
	}
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommand::Execute() {
	frc::Pose2d chassisPose = chassis->getEstimatedPose();
	frc::ChassisSpeeds robotRelativeSpeeds = chassis->getCurrentSpeeds();
	ChassisAccels robotRelativeAccels = chassis->getCurrentAccels();
	frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotRelativeSpeeds, chassisPose.Rotation());
	ChassisAccels fieldRelativeAccels = ChassisAccels::FromRobotRelativeAccels(robotRelativeAccels, chassisPose.Rotation());

	//frc::Translation2d speakerLoc = dynamicTarget.getMovingTarget(chassisPose, fieldRelativeSpeeds, fieldRelativeAccels);
	frc::Translation2d speakerLoc = dynamicTarget.getMovingTarget(chassisPose, fieldRelativeSpeeds, fieldRelativeAccels);

	frc::Translation2d chassisLoc = chassisPose.Translation();

	frc::Translation2d chassisToTarget = speakerLoc - chassisLoc;
	distance = chassisToTarget.Distance({ 0_m, 0_m });
	angle = chassisToTarget.Angle().RotateBy({ 180_deg });

	double upperAngleOffset = GetUpperAngleOffset();

	headingHelper.setTargetAngle(angle.Radians());
	double targetLowerAngle = VisionSpeakerCommandConstants::DistanceToLowerAngleTable[distance];
	double targetUpperAngle = VisionSpeakerCommandConstants::DistanceToUpperAngleTable[distance] + upperAngleOffset;
	double targetShooterVelocity = VisionSpeakerCommandConstants::DistanceToVelocityTable[distance];
	superStructure->setTargetCoord({ targetLowerAngle, targetUpperAngle });
	shooter->setTargetVelocity(targetShooterVelocity);

	units::degree_t headingTolerance = 2_deg + units::degree_t(std::clamp(1 - distance.value() / 6.0, 0.0, 1.0) * 8.0); // Heading tolerance extra of X deg when close, more precise when further back;
	units::degree_t headingError = units::math::abs(frc::InputModulus(angle.Degrees() - chassisPose.Rotation().Degrees(), -180_deg, 180_deg));

	bool lowerAngleInTolerance = std::abs(targetLowerAngle - superStructure->getLowerAngle()) < 1.0;

	bool upperAngleInTolerance = false;

	if (joystick == nullptr) {
		upperAngleInTolerance = std::abs(targetUpperAngle - superStructure->getUpperAngle()) < 1.0;
	} else {
		upperAngleInTolerance = std::abs(targetUpperAngle - superStructure->getUpperAngle()) < 3;
	}

	bool headingInTolerance = headingError < headingTolerance;
	bool shooterSpeedInTolerance = (targetShooterVelocity - 2.0) < shooter->getCurrentVelocity();

	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/LowerAngleReached", lowerAngleInTolerance);
	frc::SmartDashboard::PutNumber("VisionSpeakerCommand/Distance", distance.value());
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/UpperAngleReached", upperAngleInTolerance);
	frc::SmartDashboard::PutNumber("VisionSpeakerCommand/UpperAngleOffset", upperAngleOffset);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/HeadingReached", headingInTolerance);
	frc::SmartDashboard::PutBoolean("VisionSpeakerCommand/ShooterReached", shooterSpeedInTolerance);
	upperAngleOffsetLog.Append(upperAngleOffset);

	if (lowerAngleInTolerance && upperAngleInTolerance && headingInTolerance && shooterSpeedInTolerance) {
		if (joystick == nullptr) {
			Timer.Start();
			storage->setVoltage(StorageConstants::ScoreVolts);
		} else {
			joystick->SetRumble(frc::GenericHID::kBothRumble, 1.0);
		}
	} else {
		if (joystick != nullptr) {
			joystick->SetRumble(frc::GenericHID::kBothRumble, 0.0);
		}
	}
}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
	chassis->disableSpeedHelper();
	if (joystick == nullptr) {
		storage->setVoltage(0_V);
	} else {
		joystick->SetRumble(frc::GenericHID::kBothRumble, 0.0);
	}
}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
	if (joystick == nullptr && Timer.Get() > 0.3_s) {
		return true;
	}

	return false;
}


void VisionSpeakerCommand::SetUpperAngleOffset(double offset) {
	UPPER_ANGLE_OFFSET = offset;
};

double VisionSpeakerCommand::GetUpperAngleOffset() {
	return UPPER_ANGLE_OFFSET;
};

void VisionSpeakerCommand::ResetUpperAngleOffset() {
	UPPER_ANGLE_OFFSET = 0.0;
};
