#pragma once

#include <frc/Encoder.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/CounterBase.h>

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <cmath>
#include <frc/controller/ProfiledPIDController.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>
#include <units/base.h>
#include <units/length.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/angular_velocity.h>
#include "Constants.h"
//Motors and Encoders
#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include <Rev/CANEncoder.h>

#include <frc/AnalogPotentiometer.h>
#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <bits/stdc++.h>
#include <ctre/phoenix/sensors/CANCoder.h>

using namespace frc;
class SwerveModule{
private:
	rev::CANSparkMax RotatorMotor;
	rev::CANSparkMax DriveMotor;
	ctre::phoenix::sensors::CANCoder absEncoder;
	rev::SparkMaxRelativeEncoder * driveEncoder;
	rev::SparkMaxRelativeEncoder * turningEncoder;

	
	PIDController turningPidController;
	
	
public:
	SwerveModule(int RotatorMotorNo, int DriveMotorNo, int CANCoderId, bool reverseDirection, double Offset, bool DriveReverse, bool TurnReverse);
	
	//abs encoder vals
	double absSignum;
	double encoderOffset;

	void ResetEncoder();
	double GetCurrentPosition();
	double GetAbsEncoderPosition();
	void SetToVector(frc::SwerveModuleState& state);
	
};
