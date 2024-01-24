#include "SuperStructureCommand.h"

frc2::CommandPtr StartIntake(Intake* m_Intake, SuperStructure* m_SuperStructure, Storage* m_Storage) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(3_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ -34, -17});}, { m_SuperStructure }).ToPtr(),
		frc2::InstantCommand([m_Storage] {m_Storage->setVoltage(1_V);}, { m_Storage }).ToPtr()
	);
}

frc2::CommandPtr StopIntake(Intake* m_Intake, SuperStructure* m_SuperStructure, Storage* m_Storage) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ -34, -14});}, { m_SuperStructure }).ToPtr(),
		frc2::InstantCommand([m_Storage] {m_Storage->setVoltage(0_V);}, { m_Storage }).ToPtr()
	);
}

frc2::CommandPtr ShootingPose(Intake* m_Intake, SuperStructure* m_SuperStructure) {
	return frc2::cmd::Parallel(

		// Faltan speedwheels para shooter
		//frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, { m_Intake }).ToPtr(),
		//frc2::InstantCommand([m_SuperStructure, structureState] {m_SuperStructure->setTargetCoord({ 105, 40 });}, { m_SuperStructure }).ToPtr()
	);																			// 15 - 40
}

// frc2::CommandPtr AngleShootingProcess(double* angleShooting) {

// 	// sleep(4);
// 	// angleShooting = 15.0;
// 	// sleep(4);
// 	// angleShooting = 40.0;
// 	return;
// }

frc2::CommandPtr ShooterAngle(SuperStructure* m_SuperStructure) {
	return frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({40, 40});}, { m_SuperStructure }).ToPtr();
}

frc2::CommandPtr IdolStart(Intake* m_Intake, SuperStructure* m_SuperStructure, Storage* m_Storage, Shooter* m_shooter) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ -34, -14});}, { m_SuperStructure }).ToPtr(),
		frc2::InstantCommand([m_Storage] {m_Storage->setVoltage(0_V);}, { m_Storage }).ToPtr(),
		frc2::InstantCommand([m_shooter] {m_shooter->setVelocityVoltage(0);}, { m_shooter }).ToPtr()
	);
}