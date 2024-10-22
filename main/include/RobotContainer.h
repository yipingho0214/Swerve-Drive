#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/SwerveSubsystem.h"
#include "commands/SwerveJoystickCmd.h"
#include <frc/Joystick.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
 
  void ConfigureBindings();
  SwerveSubsystem* swerveSubsystem = new SwerveSubsystem();
  frc::Joystick* joystick = new frc::Joystick{0};
};
