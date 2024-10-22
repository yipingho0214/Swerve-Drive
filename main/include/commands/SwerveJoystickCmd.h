#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "subsystems/SwerveSubsystem.h"

class SwerveJoystickCmd
    : public frc2::CommandHelper<frc2::Command, SwerveJoystickCmd> {
 public:
  
  explicit SwerveJoystickCmd(SwerveSubsystem* s,frc::Joystick* j);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveSubsystem* swerveSubsystem;

  frc::Joystick* joystick;
};