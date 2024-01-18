#include "SuperStructureCommand.h"

frc2::CommandPtr StartIntake(Intake* m_Intake, SuperStructure* m_SuperStructure) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(3_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ 105, 45});}, { m_SuperStructure }).ToPtr()
	);
}

frc2::CommandPtr StopIntake(Intake* m_Intake, SuperStructure* m_SuperStructure) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ 0, 0 });}, { m_SuperStructure }).ToPtr()
	);
}

frc2::CommandPtr ShootingPose(Intake* m_Intake, SuperStructure* m_SuperStructure) {
	return frc2::cmd::Parallel(

		// Faltan speedwheels para shooter
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure, structureState] {m_SuperStructure->setTargetCoord({ 105, 40 });}, { m_SuperStructure }).ToPtr()
	);																			// 15 - 40
}

frc2::CommandPtr AngleShootingProcess(double* angleShooting) {

	sleep(4);
	angleShooting = 15.0;
	sleep(4);
	angleShooting = 40.0;
	return;
}