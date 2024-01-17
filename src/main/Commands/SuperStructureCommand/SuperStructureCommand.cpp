#include "SuperStructureCommand.h"

frc2::CommandPtr StartIntake(Intake* m_Intake) {
    return frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(3_V);}, {m_Intake}).ToPtr();
}
frc2::CommandPtr StopIntake(Intake* m_Intake){
    return frc2::InstantCommand([m_Intake] {m_Intake->setVoltage(0_V);}, {m_Intake}).ToPtr();

}
frc2::CommandPtr MoveStructure(SuperStructure* m_SuperStructure){
    return frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({35, 45});}, {m_SuperStructure}).ToPtr();
}
frc2::CommandPtr MoveArms(SuperStructure* m_SuperStructure){
    return frc2::InstantCommand([m_SuperStructure] {m_SuperStructure->setTargetCoord({35, 45});}, {m_SuperStructure}).ToPtr();
}