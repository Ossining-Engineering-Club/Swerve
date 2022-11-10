#include "SwerveModule.h"
#include <fmt/core.h>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
using namespace frc;
SwerveModule::SwerveModule(int RotatorMotorNo, int DriveMotorNo, int CANCoderId, 
	bool reverseDirection, double Offset, bool DriveReverse, bool TurnReverse):
RotatorMotor(RotatorMotorNo,rev::CANSparkMax::MotorType::kBrushless),
DriveMotor(DriveMotorNo, rev::CANSparkMax::MotorType::kBrushless),
absEncoder(CANCoderId),
turningPidController(KRp,KRi,KRd)
{
	encoderOffset = Offset;

	if(reverseDirection == true) absSignum = -1;
	else absSignum = 1;

	RotatorMotor.SetInverted(TurnReverse);
	DriveMotor.SetInverted(DriveReverse);
	dash -> init();

	driveEncoder = new rev::SparkMaxRelativeEncoder(DriveMotor.GetEncoder());
	turningEncoder = new rev::SparkMaxRelativeEncoder(RotatorMotor.GetEncoder());

	driveEncoder->SetPositionConversionFactor(DriveEncoderPosFactor);
	turningEncoder->SetPositionConversionFactor(turnEncoderPosFactor);
	driveEncoder->SetVelocityConversionFactor(DriveEncoderPosFactor);
	turningEncoder->SetVelocityConversionFactor(turnEncoderPosFactor);

    turningPidController.EnableContinuousInput(-M_PI,M_PI);
	SwerveModule::ResetEncoder();
}
void SwerveModule::setOffset(){
	offset = SwerveModule::GetAbsEncoderPosition();
}
double SwerveModule::GetCurrentPosition(){
	return turningEncoder->GetPosition()-offset; //(fmod(turningEncoder->GetPosition(),2*M_PI);//-M_PI);
}

//Radians
double SwerveModule::GetAbsEncoderPosition(){
	double ang = absEncoder.GetAbsolutePosition(); //Default Position is in degrees
	
	ang *= (M_PI / 180.0);
	ang -= encoderOffset;//accounts for offset
	if ( ang < -M_PI) ang += 2 * M_PI;
	else if (ang > M_PI) ang -= 2*M_PI;
	return ang; //* absSignum;
}

void SwerveModule::ResetEncoder(){
	driveEncoder->SetPosition(0);
	turningEncoder->SetPosition(0);//(SwerveModule::GetAbsEncoderPosition()/(2.0*M_PI)));//GetAbsEncoderPosition());
}

//1 = drive 0= rotator
double SwerveModule::GetDrivePower(frc::SwerveModuleState& state){
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	return optimizedstate.speed*(1/(MAXMotorSPEED*1_mps));
}

double SwerveModule::GetRotatorPower(frc::SwerveModuleState& state){
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	double turningVal;
	if(turningPidController.Calculate(SwerveModule::GetCurrentPosition(),optimizedstate.angle.Radians().value())>1)
		turningVal=1;
	else if(turningPidController.Calculate(SwerveModule::GetCurrentPosition(),optimizedstate.angle.Radians().value())<-1)
		turningVal=-1;
	else
		turningVal = turningPidController.Calculate(SwerveModule::GetCurrentPosition(),optimizedstate.angle.Radians().value());
	return turningVal;
}

//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//stops automatic recentering
	if(std::abs(state.speed.value()) > 0.001){
		DriveMotor.Set(.2*GetDrivePower(state));
		RotatorMotor.Set(.4*GetRotatorPower(state));
	}
	else{
		DriveMotor.Set(0);
		RotatorMotor.Set(0);
	}
	
}
