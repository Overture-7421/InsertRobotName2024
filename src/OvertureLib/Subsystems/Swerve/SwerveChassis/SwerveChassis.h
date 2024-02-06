// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/filter/LinearFilter.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc2/command/SubsystemBase.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include "OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h"
#include "OvertureLib/Math/ChassisAccels.h"

using namespace pathplanner;

class SwerveChassis : public frc2::SubsystemBase {
public:
	SwerveChassis();
	void setModulePositions(std::array<frc::Translation2d, 4>* positions);
	void setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter);
	void setModules(SwerveModule* frontLeft, SwerveModule* frontRight, SwerveModule* backleft, SwerveModule* backRight);
	void setRotatorPID(double kP, double kI, double kD);
	void setDrivePID(double kP, double kI, double kD);
	void setFeedForward(units::volt_t kS, units::volt_t kV, units::volt_t kA);

	void driveRobotRelative(frc::ChassisSpeeds speeds);
	void driveFieldRelative(frc::ChassisSpeeds speeds);
	frc::ChassisSpeeds getRobotRelativeSpeeds();
	frc::ChassisSpeeds getFieldRelativeSpeeds();
	ChassisAccels getFIeldRelativeAccels();

	frc::Pose2d getOdometry();
	void resetOdometry(frc::Pose2d initPose);
	const frc::SwerveDriveKinematics<4>& getKinematics();
	void addVisionMeasurement(frc::Pose2d pose, units::second_t Latency);
	void resetAngle(double angle = 0);

	void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
	wpi::array<frc::SwerveModuleState, 4> getModuleStates();
	wpi::array<frc::SwerveModulePosition, 4> getModulePosition();

	double getPitch();
	double getYaw();
	double getRoll();

	void updateOdometry();
	void shuffleboardPeriodic();

protected:
	double linearX;
	double linearY;
	double angular;

	OverPigeon* pigeon;

	SwerveModule* frontLeftModule;
	SwerveModule* frontRightModule;
	SwerveModule* backLeftModule;
	SwerveModule* backRightModule;

	frc::SwerveDriveKinematics<4>* kinematics;

	std::array<frc::SwerveModulePosition, 4>* odometryPos;

	frc::SwerveDrivePoseEstimator<4>* odometry;

	frc::ChassisSpeeds fieldRelativeSpeed, lastFieldRelativeSpeed;
	ChassisAccels fieldRelativeAccel;
	frc::LinearFilter<units::meters_per_second_squared_t> accelXFilter = frc::LinearFilter<units::meters_per_second_squared_t>::SinglePoleIIR(0.05, 0.02_s);
	frc::LinearFilter<units::meters_per_second_squared_t> accelYFilter = frc::LinearFilter<units::meters_per_second_squared_t>::SinglePoleIIR(0.05, 0.02_s);
	frc::LinearFilter<units::radians_per_second_squared_t> accelOmegaFilter = frc::LinearFilter<units::radians_per_second_squared_t>::SinglePoleIIR(0.05, 0.02_s);

private:

	frc::Field2d field2d;
};