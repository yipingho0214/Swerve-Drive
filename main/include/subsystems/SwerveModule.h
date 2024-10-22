#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "frc/controller/PIDController.h"
#include <frc/kinematics/SwerveModuleState.h>

class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int driveID,int angleID,int encoderID,
                double p,double i,double d,int Reversed,int start);

  void Periodic() override;

  void SimulationPeriodic() override;

  //set drive's speed ; speed = 0 ~ 1
  void set_speed(double speed);

  //set angle ; target_angle = 0 ~ 4095
  void set_angle(int target_angle);

   //set drive and angle's speed 0
  void stop();


 private:
  TalonFX* drive_motor;
  TalonFX* angle_motor;

  TalonSRX* encoder;

  frc::PIDController* pidController;

  int reversed;
  int encoder_start;

};