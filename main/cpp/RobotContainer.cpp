#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h> 

RobotContainer::RobotContainer() {

  swerveSubsystem->SetDefaultCommand(SwerveJoystickCmd(swerveSubsystem,joystick));
  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  
}
