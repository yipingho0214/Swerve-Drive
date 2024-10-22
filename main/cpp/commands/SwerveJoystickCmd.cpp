#include "commands/SwerveJoystickCmd.h"
#include <iostream>
#include <cmath>

using namespace std;

SwerveJoystickCmd::SwerveJoystickCmd(SwerveSubsystem* s,frc::Joystick* j){
    joystick = j;
    swerveSubsystem = s;
    AddRequirements(swerveSubsystem);
}

void SwerveJoystickCmd::Initialize(){
    swerveSubsystem->gyro.Calibrate(); 
    swerveSubsystem->gyro.Reset(); 
}

void SwerveJoystickCmd::Execute(){
  double x = joystick->GetX();
  double y = joystick->GetY();
  double z = joystick->GetZ();

  double hypotenuse = sqrt(x*x + y*y); 

  if(z<-0.3&&hypotenuse<0.3){
    swerveSubsystem->left_rotation_angle();
    swerveSubsystem->set_speed(std::abs(z)*0.4);
    return;
  }
  if(z>0.3&&hypotenuse<0.3){
    swerveSubsystem->right_rotation_angle();
    swerveSubsystem->set_speed(std::abs(z)*0.4);
    return;
  }

  if(hypotenuse<0.1){
    swerveSubsystem->stop();
    return;
  }

  double sin = x * -1 / hypotenuse;
  double cos = y * -1 / hypotenuse;

  double rad_sin = asin(sin);
  double rad_cos = acos(cos);

  double turn_angle;

  if(rad_sin>=0&&rad_cos<=M_PI/2){
    turn_angle = rad_sin;
  }
  else if(rad_sin<0&&rad_cos<M_PI/2){
    turn_angle = 2 * M_PI - rad_cos;
  }
  else if(rad_sin>0&&rad_cos>M_PI/2){
    turn_angle = M_PI - rad_sin;
  }
  else if(rad_sin<=0&&rad_cos>=M_PI/2){
    turn_angle = M_PI + std::abs(rad_sin);
  }

  turn_angle = turn_angle / M_PI * 2048;

  if(z<-0.3){
    swerveSubsystem->left_rotation_drive(turn_angle,std::abs(z)*0.45);
    return;
  }
  if(z>0.3){
    swerveSubsystem->right_rotation_drive(turn_angle,std::abs(z)*0.45);
    return;
  }

  if(!std::isnan(turn_angle)){
    swerveSubsystem->set_angle(turn_angle);
  }

  swerveSubsystem->set_speed(hypotenuse*0.8);
}

void SwerveJoystickCmd::End(bool interrupted){}

bool SwerveJoystickCmd::IsFinished(){
  return false;
}