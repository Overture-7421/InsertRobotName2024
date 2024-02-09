#include "WaitForCheckPoint.h"

frc2::CommandPtr WaitForButton(frc::XboxController* controller, int buttonNumber) {
	return frc2::cmd::WaitUntil([=]() {return controller->GetRawButton(buttonNumber);});
}
