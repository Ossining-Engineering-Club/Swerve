#include "SwerveModule.h"
#include <fmt/core.h>
#include <cmath>


#include <frc/smartdashboard/SmartDashboard.h>
using namespace frc;
SwerveModule::SwerveModule(int RotatorMotorNo, int DriveMotorNo, int CANCoderId, bool reverseDirection, double Offset, bool DriveReverse,
						   bool TurnReverse):
RotatorMotor(RotatorMotorNo,rev::CANSparkMax::MotorType::kBrushless),
DriveMotor(DriveMotorNo, rev::CANSparkMax::MotorType::kBrushless),
absEncoder(CANCoderId),
turningPidController(KRp,KRi,KRd)
{
	encoderOffset = Offset;

	if(reverseDirection == true)
	absSignum = -1;
	else
	absSignum = 1;

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
double SwerveModule::GetCurrentPosition(){
	return turningEncoder->GetPosition();//(fmod(turningEncoder->GetPosition(),2*M_PI);//-M_PI);
}
//Radians
double SwerveModule::GetAbsEncoderPosition(){
	double ang = absEncoder.GetPosition(); //check this
	ang = fmod(ang,360);
	ang *= (M_PI / 180.0);
	//ang = fabs(ang);
	//ang = fmod(ang,M_2_PI); //precentage of a full rotation
	ang -= encoderOffset;//accounts for offset
	return fabs(ang); //* absSignum;
}
void SwerveModule::ResetEncoder(){
	driveEncoder->SetPosition(0);
	turningEncoder->SetPosition(SwerveModule::GetAbsEncoderPosition());//GetAbsEncoderPosition());
}
//1 = drive 0= rotator
double SwerveModule::GetDrivePower(frc::SwerveModuleState& state){
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	//auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
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
	//auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	return turningVal;
}
//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//stops automatic recentering
	if(std::abs(state.speed.value()) > 0.001){
		//auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
		DriveMotor.Set(.2*GetDrivePower(state));
		//dash->PutNumber("DrivePower",(optimizedstate.speed*(1/(MAXMotorSPEED*1_mps))));
		//dash->PutNumber("angleoptim",optimizedstate.angle.Radians().value());
		//dash->PutNumber("RotatorPower",turningPidController.Calculate(SwerveModule::GetCurrentPosition(),optimizedstate.angle.Radians().value()));
		RotatorMotor.Set(.4*GetRotatorPower(state));
	}
	else{
		DriveMotor.Set(0);
		RotatorMotor.Set(0);
		//return;//check this
	}
	
}
