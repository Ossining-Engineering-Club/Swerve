#include "SwerveModule.h"
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
using namespace frc;
SwerveModule::SwerveModule(int RotatorPort, int DrivePort, int EncoderPort1, int reverseDirection, double Offset):
RotatorMotor(RotatorPort),
DriveMotor(DrivePort),
RotatorEncoder(EncoderPort1,360,Offset),
pidController(.4,0,.0001)

{
	
	correction = 0.0;
	signum = reverseDirection;
	//Limits PID range -pi and pi(find out why these numbers are specified(read on internet))
	//Look over the range
    pidController.EnableContinuousInput(0,2*3.1415926535);
	MotorIsForward = true;
	
}
//ANGLE RETURNED IN RADIANS!!!!!!!!!!!!!!!!
double SwerveModule::GetCurrentPosition(){
	double angle = RotatorEncoder.Get();
	while(angle < 0){
		angle+=360;
	}	
	while(angle > 360){
		angle-=360;
	}

	if (angle == 0.0)
		angle = 360.0;
	return angle*(M_PI/180);
}

double SwerveModule::GetTurningEncoderPosition(){
	double ang = RotatorEncoder.Get();
	while(ang < 0){
		ang+=360;
	}	
	while(ang > 360){
		ang-=360;
	}
	return ang;
}

//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//Optimised state to stop from spinning more than pi/2 radians
	auto driveOut = (state.speed*(1/(MAXSPEED*1_mps))*signum);
/*	if(state.angle.Radians().value()>=(M_PI/2)&&state.angle.Radians().value()<=((3*M_PI)/2)){
		DriveMotor.Set(-driveOut);
	}else{
		DriveMotor.Set(driveOut);
	}*/
	DriveMotor.Set(driveOut);
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	
	//auto rotOut = turningPID.Calculate(SwerveModule::GetCurrentPosition()*1_rad);
	double setpoint = optimizedstate.angle.Radians().value(); // oOptimization offset can be calculated here.
   /* if (setpoint < 0) {
        setpoint = MAXVOLTAGE + setpoint;
    }
    if (setpoint > MAXVOLTAGE) {
        setpoint = setpoint - MAXVOLTAGE;
    }*/
	double rot = std::clamp(pidController.Calculate(SwerveModule::GetCurrentPosition(), setpoint), -1.0, 1.0);
   
	rotate = rot;
	RotatorMotor.Set(rot);
}
//Not used, currently using Optimize function to avoid spining over 90 degrees 
/*
double SwerveModule::CalculateError(double TargetAngle){

	double CDist = fmod((GetCurrentPosition()+((2*M_PI)-TargetAngle)),(2*M_PI));
	double error;
	double target = TargetAngle;
	if (target == 0)
		target = (2*M_PI);
	if(CDist <= (M_PI_2)){
		error = CDist;
		MotorIsForward = true;
	}
	else if(CDist <= (3*M_PI_2)){
		error = CDist-180.0;
		MotorIsForward = false;
	}
	else if(CDist <=  (2*M_PI)){
		error = CDist -  (2*M_PI);
		MotorIsForward = true;
	}
	else{
		error = 0.0;
	}

	return error;
}
*/
