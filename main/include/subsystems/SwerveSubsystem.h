#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/SwerveModule.h"
#include "AHRS.h"

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  void Periodic() override;

  void SimulationPeriodic() override;

  //set all drive motor speed ; speed = 0 ~ 1
  void set_speed(double speed);

  //set all angle motor angle + gyro offset ; angle = 0 ~ 4095
  void set_angle(int angle);

  //set all drive and angle motor's speed 0
  void stop();

  //get an angle added gyro offset ; angle = 0 ~ 4095 ; return 0 ~ 4095
  int add_gyro_offset(int angle);

  //left rotation
  void left_rotation_angle();
  //right rotation
  void right_rotation_angle();

  //left rotation and drive ; angle = 0 ~ 4095 ; speed = 0 ~ 1
  void left_rotation_drive(int angle,double speed);
  //right rotation and drive ; angle = 0 ~ 4095 ; speed = 0 ~ 1
  void right_rotation_drive(int angle,double speed);

  AHRS gyro{frc::SPI::Port::kMXP};
  
 private:
  

  SwerveModule frontright   {1,2,9, 0.0005, 0.0005,0.00000125,1,2021};
  SwerveModule frontleft    {3,4,10,0.00045,0,     0.0000001, 1,2828};
  SwerveModule backright    {5,6,11,0.0004, 0,     0.000002,  1,1759};    
  SwerveModule backleft     {7,8,12,0.0005, 0,     0.0000011, 1,2671};

};