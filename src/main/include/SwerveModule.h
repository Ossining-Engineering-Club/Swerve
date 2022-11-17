#pragma once
#include <frc/motorcontrol/Spark.h>
#include <frc/CounterBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/angle.h>
#include <units/base.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include <Rev/CANEncoder.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class SwerveModule{
  public:
	SwerveModule(int RotatorMotorNo, 
		int DriveMotorNo, int CANCoderId, bool reverseDirection, 
		double Offset, bool DriveReverse, bool TurnReverse);
	frc::SwerveModuleState GetState() const;	
	double GetDrivePower();
	double GetRotatorPower();
	void ResetEncoder();
	double GetCurrentAngle();
	double GetAbsEncoderAngle();
	void SetToVector(frc::SwerveModuleState& state);

  private:
	rev::CANSparkMax RotatorMotor;
	rev::CANSparkMax DriveMotor;
	ctre::phoenix::sensors::CANCoder absEncoder;
	rev::SparkMaxRelativeEncoder * driveEncoder;
	rev::SparkMaxRelativeEncoder * turningEncoder;
	double absSignum; 		//abs encoder vals
	double absEncoderOffset;  //Set on init from constants
	double turningEncoderOffset;
	frc2::PIDController drivePIDController{KDP, KDI, KDD};
  	frc::ProfiledPIDController<units::radians>
	  turningPIDController{KRP, KRI, KRD,
	  {MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION}};
	frc::SimpleMotorFeedforward<units::meters>
	 	drivingFeedforward{1_V, 3_V / 1_mps};
  	frc::SimpleMotorFeedforward<units::radians> 
	  	turningFeedforward{1_V, 0.5_V / 1_rad_per_s};
};
