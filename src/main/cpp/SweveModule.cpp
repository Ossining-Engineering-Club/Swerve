#include "SwerveModule.h"
#include <fmt/core.h>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
using namespace frc;
SwerveModule::SwerveModule(int RotatorMotorNo, 
	int DriveMotorNo, int CANCoderId, bool reverseDirection, 
	double absEncoderOffsetConstant, bool DriveReverse, bool TurnReverse):
RotatorMotor(RotatorMotorNo,rev::CANSparkMax::MotorType::kBrushless),
DriveMotor(DriveMotorNo, rev::CANSparkMax::MotorType::kBrushless),
absEncoder(CANCoderId),
turningPidController(KRp,KRi,KRd)
{
	absEncoderOffset = absEncoderOffsetConstant; //Assign from constants Table
	if(reverseDirection == true) absSignum = -1.0;
	else absSignum = 1.0;
	RotatorMotor.SetInverted(TurnReverse);
	DriveMotor.SetInverted(DriveReverse);
	driveEncoder = new rev::SparkMaxRelativeEncoder(DriveMotor.GetEncoder());
	turningEncoder = new rev::SparkMaxRelativeEncoder(RotatorMotor.GetEncoder());
	driveEncoder->SetPositionConversionFactor(DriveEncoderPosFactor);
	turningEncoder->SetPositionConversionFactor(turnEncoderPosFactor);
	driveEncoder->SetVelocityConversionFactor(DriveEncoderPosFactor);
	turningEncoder->SetVelocityConversionFactor(turnEncoderPosFactor);
    turningPidController.EnableContinuousInput(-M_PI,M_PI);
	SwerveModule::ResetEncoder();
}
//Get position from Motor encoder subtracting Offest seen on absolute encoder
double SwerveModule::GetCurrentPosition(){
	return turningEncoder->GetPosition()-turningEncoderOffset; 
}
//Return the Absolute Encoder Angle between -Pi and +PI
double SwerveModule::GetAbsEncoderPosition(){
	double ang = absEncoder.GetAbsolutePosition(); //Units are degrees
	ang *= (M_PI / 180.0);	 		// Convert to Radians
	ang -= absEncoderOffset;    	// Subtract offset from Constants
	// Ensure value is between -PI and +PI
	if ( ang < -M_PI) ang += 2.0 * M_PI;
	else if (ang > M_PI) ang -= 2.0 * M_PI;
	return ang; 
}
//Get Absolute Encoder Offset and Reset Encoder positions for relative Enciders
void SwerveModule::ResetEncoder(){
	turningEncoderOffset = SwerveModule::GetAbsEncoderPosition();
	driveEncoder->SetPosition(0.0);
	turningEncoder->SetPosition(0.0);
}

//1 = drive 0= rotator
double SwerveModule::GetDrivePower(frc::SwerveModuleState& state){
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	return optimizedstate.speed*(1.0/(MAXMotorSPEED*1_mps));
}

double SwerveModule::GetRotatorPower(frc::SwerveModuleState& state){
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	double turningVal = turningPidController.Calculate(
			SwerveModule::GetCurrentPosition(),optimizedstate.angle.Radians().value());
	if (turningVal > 1.0) turningVal = 1.0;
	else if (turningVal < -1.0) turningVal = -1.0;
	return turningVal;
}

//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//stops automatic recentering
	if(std::abs(state.speed.value()) > 0.001){
		DriveMotor.Set(0.2*GetDrivePower(state));
		RotatorMotor.Set(0.4*GetRotatorPower(state));
	}
	else{
		DriveMotor.Set(0.0);
		RotatorMotor.Set(0.0);
	}
	
}
