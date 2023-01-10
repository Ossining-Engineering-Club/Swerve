#include "SwerveModule.h"

SwerveModule::SwerveModule(int RotatorMotorNo, 
	int DriveMotorNo, int CANCoderId, bool reverseDirection, 
	double absEncoderOffsetConstant, bool DriveReverse, bool TurnReverse):
RotatorMotor(RotatorMotorNo,rev::CANSparkMax::MotorType::kBrushless),
DriveMotor(DriveMotorNo, rev::CANSparkMax::MotorType::kBrushless),
absEncoder(CANCoderId)
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
	driveEncoder->SetVelocityConversionFactor(DriveEncoderVelocityFactor);
	turningEncoder->SetVelocityConversionFactor(turnEncoderVelocityFactor);
    turningPIDController.EnableContinuousInput(
      -1.0*std::numbers::pi, 
	  std::numbers::pi);
	SwerveModule::ResetEncoder();
}
// Get State to be used for odometry
frc::SwerveModuleState SwerveModule::GetState() const 
{
  return {units::meters_per_second_t{driveEncoder->GetVelocity()},
          frc::Rotation2d(units::radian_t((turningEncoder->GetPosition())-turningEncoderOffset))};
}
//Get wheel angle from Motor encoder subtracting Offest seen on absolute encoder
double SwerveModule::GetCurrentAngle(){
	return turningEncoder->GetPosition()-turningEncoderOffset; 
}
frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{driveEncoder -> GetPosition()},
          units::radian_t{turningEncoder -> GetPosition()}};
}
//Return the Absolute Encoder Angle between -Pi and +PI
double SwerveModule::GetAbsEncoderAngle(){
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
	turningEncoderOffset = SwerveModule::GetAbsEncoderAngle();
	driveEncoder->SetPosition(0.0);
	turningEncoder->SetPosition(0.0);
}

double SwerveModule::GetDrivePower(){
	return (driveEncoder->GetVelocity());
}

double SwerveModule::GetRotatorPower(){
  	return (turningEncoder->GetVelocity());
}


//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//stops automatic recentering
	if(std::abs(state.speed.value()) > 0.001){
	frc::SwerveModuleState optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentAngle()*1_rad));

	double turningVal = turningPIDController.Calculate(
			SwerveModule::GetCurrentAngle(),optimizedstate.angle.Radians().value());
	if (turningVal > 1.0) turningVal = 1.0;
	else if (turningVal < -1.0) turningVal = -1.0;
	
	
		DriveMotor.Set(0.4*optimizedstate.speed*(1.0/(MAX_SPEED*1_mps)));
		RotatorMotor.Set(0.7*turningVal);
	}
	else{
		DriveMotor.Set(0.0);
		RotatorMotor.Set(0.0);
	}
}
/*
//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& referenceState){
  // 1. Optimize reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(GetCurrentAngle()));

  // 2. Calculate drive output from the drive PID controller.
  const auto driveOutput = drivePIDController.Calculate(
     driveEncoder->GetVelocity(), state.speed.value());

  // 3. Calculate the voltage needed to drive the moter to get the desired velocity
  const auto driveFeedforward = drivingFeedforward.Calculate(state.speed);

  // 4. Calculate turning motor output from the turning PID controller.
  const auto turnOutput = turningPIDController.Calculate(
      units::radian_t(GetCurrentAngle()), state.angle.Radians());
  
  // 5. Calculate the voltage needed to drive the moter to get the desired turn rate
  const auto turnFeedforward = turningFeedforward.Calculate(
      turningPIDController.GetSetpoint().velocity);

  // 6. Set the motor outputs.
  if ((units::volt_t{driveOutput} + driveFeedforward) > units::volt_t{(12.0*0.001)})
  {
  	DriveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  	RotatorMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
  }  
  //stops automatic recentering
  else{
  	DriveMotor.SetVoltage(units::volt_t{0.0});
	RotatorMotor.SetVoltage(units::volt_t{0.0});
  }
	
}
*/