#include "subsystems/SwerveSubsystem.h"
#include <iostream>

using namespace std;

SwerveSubsystem::SwerveSubsystem() {
    
}

void SwerveSubsystem::Periodic() {}

void SwerveSubsystem::SimulationPeriodic() {}

void SwerveSubsystem::set_speed(double speed){
    frontright.set_speed(speed);
    frontleft.set_speed(speed);
    backright.set_speed(speed);
    backleft.set_speed(speed);
}

void SwerveSubsystem::set_angle(int angle){

    int final_angle = add_gyro_offset(angle);

    frontright.set_angle(final_angle);
    frontleft.set_angle(final_angle);
    backright.set_angle(final_angle);
    backleft.set_angle(final_angle);
}

void SwerveSubsystem::stop(){
    frontright.stop();
    frontleft.stop();
    backright.stop();
    backleft.stop();
}

int SwerveSubsystem::add_gyro_offset(int angle){
    int gyro_offset = gyro.GetAngle() * 4096 / 360;
    gyro_offset %= 4096;

    angle += gyro_offset;

    if(angle>=4096) angle-=4096;
    if(angle<0) angle+=4096;

    return angle;
}

void SwerveSubsystem::left_rotation_angle(){
    frontright.set_angle(512);
    frontleft.set_angle(1536);
    backright.set_angle(3584);
    backleft.set_angle(2560);
}
void SwerveSubsystem::right_rotation_angle(){
    frontright.set_angle(2560);
    frontleft.set_angle(3584);
    backright.set_angle(1536);
    backleft.set_angle(512);
}

void SwerveSubsystem::left_rotation_drive(int angle,double speed){
    int offset = 4096 - add_gyro_offset(0) - angle;
    if(offset<0) offset+=4096;

    int frontright_angle;
    int frontleft_angle;
    int backright_angle;
    int backleft_angle;

    if(offset<=1024){
        backright_angle=3584;
        backleft_angle = 3584-offset;

        frontleft_angle = 512+offset*2;

        frontright_angle = 512-offset;
        if(frontright_angle<0) frontright_angle+=4096;
    }
    else if(offset<=2048){
        offset-=1024;

        backleft_angle = 2560;
        frontleft_angle = 2560 - offset;

        frontright_angle = 3584 + offset*2;
        if(frontright_angle>=4096) frontright_angle-=4096;

        backright_angle = 3584 - offset;
    }
    else if(offset<=3072){
        offset-=2048;

        frontleft_angle = 1536;
        frontright_angle = 1536 - offset;

        backright_angle = 2560  + offset*2;
        if(backright_angle>=4096) backright_angle-=4096;

        backleft_angle = 2560 - offset;
    }
    else{
        offset-=3072;

        frontright_angle = 512;

        backright_angle = 512 - offset;
        if(backright_angle<0) backright_angle+=4096;

        backleft_angle = 1536  + offset*2;

        frontleft_angle = 1536 - offset;
    }

    frontright.set_angle(frontright_angle);
    frontleft.set_angle(frontleft_angle);
    backright.set_angle(backright_angle);
    backleft.set_angle(backleft_angle);

    frontright.set_speed(speed);
    frontleft.set_speed(speed);
    backright.set_speed(speed);
    backleft.set_speed(speed);
}
void SwerveSubsystem::right_rotation_drive(int angle,double speed){
    int offset = add_gyro_offset(0) - angle;
    if(offset<0) offset+=4096;

    int frontright_angle;
    int frontleft_angle;
    int backright_angle;
    int backleft_angle;

    if(offset<=1024){
        backleft_angle = 512;
        backright_angle = 512 + offset;

        frontright_angle = 3584 - offset * 2;

        frontleft_angle = 3584 + offset;
        if(frontleft_angle>4096) frontleft_angle -= 4096;
    }
    else if(offset<=2048){
        offset-=1024;
        
        backright_angle = 1536;
        frontright_angle = 1536 + offset;

        frontleft_angle = 512 - offset * 2;
        if(frontleft_angle<0) frontleft_angle+=4096;

        backleft_angle = 512 + offset;
    }
    else if(offset<=3072){
        offset-=2048;
        
        frontright_angle = 2560;
        frontleft_angle = 2560 + offset;

        backleft_angle = 1536 - offset * 2;
        if(backleft_angle<0) backleft_angle+=4096;

        backright_angle = 1536 + offset;
    }
    else{
        offset-=3072;
        
        frontleft_angle = 3584;

        backleft_angle = 3584 + offset;
        if(backleft_angle>4096) backleft_angle -= 4096;

        backright_angle = 2560 - offset * 2;
        frontright_angle = 2560 + offset;
    }

    frontright.set_angle(frontright_angle);
    frontleft.set_angle(frontleft_angle);
    backright.set_angle(backright_angle);
    backleft.set_angle(backleft_angle);

    frontright.set_speed(speed);
    frontleft.set_speed(speed);
    backright.set_speed(speed);
    backleft.set_speed(speed);
}