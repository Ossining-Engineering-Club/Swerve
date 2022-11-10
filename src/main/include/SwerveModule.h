#pragma once
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/CounterBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <units/angle.h>
#include <units/base.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "Constants.h"

//Motors and Encoders
#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include <Rev/CANEncoder.h>
#include <ctre/phoenix/sensors/CANCoder.h>

using namespace frc;
class SwerveModule{
private:
	rev::CANSparkMax RotatorMotor;
	rev::CANSparkMax DriveMotor;
	ctre::phoenix::sensors::CANCoder absEncoder;
	rev::SparkMaxRelativeEncoder * driveEncoder;
	rev::SparkMaxRelativeEncoder * turningEncoder;
		double offset;	
	SmartDashboard* dash;
	PIDController turningPidController;
	
public:
	SwerveModule(int RotatorMotorNo, int DriveMotorNo, int CANCoderId, bool reverseDirection, double Offset, bool DriveReverse, bool TurnReverse);
	void setOffset();
	//abs encoder vals
	double absSignum;
	double encoderOffset;
	double GetDrivePower(frc::SwerveModuleState& state);
	double GetRotatorPower(frc::SwerveModuleState& state);
	void ResetEncoder();
	double GetCurrentPosition();
	double GetAbsEncoderPosition();
	void SetToVector(frc::SwerveModuleState& state);
	
};
