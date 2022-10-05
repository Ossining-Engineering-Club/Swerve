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

	RotatorMotor.SetInverted(DriveReverse);
	DriveMotor.SetInverted(TurnReverse);

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
	return turningEncoder->GetPosition();
}
//Radians
double SwerveModule::GetAbsEncoderPosition(){
	double ang = absEncoder.GetPosition(); //check this
	ang *= 2*M_PI; //precentage of a full rotation
	ang -= encoderOffset;//accounts for offset
	return ang * absSignum;
}
void SwerveModule::ResetEncoder(){
	driveEncoder->SetPosition(0);
	turningEncoder->SetPosition(GetAbsEncoderPosition());
}
//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//stops automatic recentering
	if(std::abs(state.speed.value()) > 0.001){
		auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
		DriveMotor.Set(state.speed*(1/(MAXSPEED*1_mps)));
		RotatorMotor.Set(turningPidController.Calculate(SwerveModule::GetCurrentPosition(),optimizedstate.angle.Radians().value()));
	}
	else{
		DriveMotor.Set(0);
		RotatorMotor.Set(0);
		return;//check this
	}
	
}
