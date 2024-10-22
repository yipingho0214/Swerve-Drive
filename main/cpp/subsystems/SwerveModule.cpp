#include "subsystems/SwerveModule.h"
#include <iostream>

using namespace std;

SwerveModule::SwerveModule(int driveID,int angleID,int encoderID,
                            double p,double i,double d,int Reversed,int start) {
    drive_motor = new TalonFX{driveID};
    angle_motor = new TalonFX{angleID};

    encoder = new TalonSRX{encoderID};
    encoder->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

    pidController = new frc::PIDController{p,i,d};

    reversed = Reversed;

    encoder_start = start;
}

void SwerveModule::set_speed(double speed){
    drive_motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed*reversed);
}

void SwerveModule::set_angle(int target_angle){
    pidController->Reset();

    int current = int(encoder->GetSelectedSensorPosition(0)-encoder_start) % 4096;
    if(reversed==-1) current+=2048;
    if(current<0) current+=4096;
    if(current>=4096) current-=4096;

    if(abs(target_angle-current)>2048){
        if(target_angle>current) target_angle -= 4096;
        else target_angle += 4096;
    }

    if(abs(target_angle-current)>1024){
        if(target_angle>current) target_angle -= 1024;
        else target_angle += 1024;
        reversed *= -1;
    }

    double output = pidController->Calculate(current, target_angle);
    
    angle_motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output*-1);
}

void SwerveModule::stop(){
    drive_motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    angle_motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}

void SwerveModule::Periodic() {}

void SwerveModule::SimulationPeriodic() {}
