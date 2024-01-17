#include "SuperStructureCommand.h"

frc2::CommandPtr StartIntake(Intake* m_Intake, SuperStructure* m_SuperStructure) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(3_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ 30, -15 });}, { m_Intake }).ToPtr()
	);
}

frc2::CommandPtr StopIntake(Intake* m_Intake, SuperStructure* m_SuperStructure) {
	return frc2::cmd::Parallel(
		frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, { m_Intake }).ToPtr(),
		frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({ 0, 0 });}, { m_Intake }).ToPtr()
	);
}