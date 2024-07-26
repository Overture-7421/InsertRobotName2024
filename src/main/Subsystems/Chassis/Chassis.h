// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h>
#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include "Constants.h"

class Chassis : public SwerveChassis {
public:
	Chassis();
	void setAllianceColor();
	void driveFieldRelative(frc::ChassisSpeeds speeds);

	frc::Rotation2d getRotation2d() override {
		return chassisPigeon.GetRotation2d();
	}
	frc::Rotation3d getRotation3d() override {
		return chassisPigeon.GetRotation3d();
	}

	units::velocity::meters_per_second_t getMaxModuleSpeed() override {
		return ChassisConstants::MaxModuleSpeed;
	}

	units::length::meter_t getDriveBaseRadius() override {
		return ChassisConstants::DriveBaseRadius;
	}

protected:
	frc::SwerveDriveKinematics<4>& getKinematics() override {
		return kinematics;
	}

	SwerveModule& getFrontRightModule() override {
		return frontRight;
	}

	SwerveModule& getFrontLeftModule() override {
		return frontLeft;
	}

	SwerveModule& getBackRightModule() override {
		return backRight;
	}

	SwerveModule& getBackLeftModule() override {
		return backLeft;
	}

	frc::SlewRateLimiter<units::velocity::meters_per_second>& getVyLimiter() {
		return yLimiter;
	}

	frc::SlewRateLimiter<units::velocity::meters_per_second>& getVxLimiter() {
		return xLimiter;
	}

	frc::SlewRateLimiter<units::angular_velocity::radians_per_second>& getVwLimiter() {
		return rLimiter;
	}

	wpi::log::StructLogEntry<frc::Pose2d>& getPoseLog() {
		return poseLog;
	}

	wpi::log::StructLogEntry<frc::Pose2d>& getVisionPoseLog() {
		return visionPoseLog;
	}

private:
	SwerveModule frontLeft{ChassisConstants::GetFrontLeftModuleConfig()};
	SwerveModule frontRight{ChassisConstants::GetFrontRightModuleConfig()};
	SwerveModule backLeft{ChassisConstants::GetBackLeftModuleConfig()};
	SwerveModule backRight{ChassisConstants::GetBackRightModuleConfig()};

	OverPigeon chassisPigeon{ 13, "OverCANivore" };

	frc::SlewRateLimiter<units::meters_per_second> xLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::meters_per_second> yLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::radians_per_second> rLimiter{ 18_tr_per_s_sq };

	wpi::log::DataLog& log = frc::DataLogManager::GetLog();
	wpi::log::StructLogEntry<frc::Pose2d> poseLog = wpi::log::StructLogEntry<frc::Pose2d>(log, "/swerve/pose");
	wpi::log::StructLogEntry<frc::Pose2d> visionPoseLog = wpi::log::StructLogEntry<frc::Pose2d>(log, "/swerve/vision_pose");

	frc::SwerveDriveKinematics<4> kinematics{
		ChassisConstants::FrontLeftModuleTranslation,   //Front Left
		ChassisConstants::FrontRightModuleTranslation,   //Front Right
		ChassisConstants::BackRightModuleTranslation,   //Back Right
		ChassisConstants::BackLeftModuleTranslation,   //Back Left
	};

	int m_AllianceColor = 0;
};
