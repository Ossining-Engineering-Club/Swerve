#pragma once
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/ADXRS450_Gyro.h>
#include "SwerveModule.h"
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "Constants.h"
using namespace frc;

//KnO stands for Kinematics and Odometry
class Drivetrain{
    public:
        Drivetrain(
            units::length::meter_t startingx,
            units::length::meter_t startingy, 
            units::angle::radian_t startingangle);
        void Drive(
            units::velocity::meters_per_second_t xspeed,
            units::velocity::meters_per_second_t yspeed,
            units::radians_per_second_t angularVelocity,
            bool FieldOriented);
        void UpdateOdometry();
        void SwerveOdometryGetPose(units::angle::radian_t gyroAngle);
        void ResetDrive();
        double GetGyro();
        double GetValue(int swerve_module, int readItem );
        //States
        SwerveModuleState frontLeft;
        SwerveModuleState frontRight;
        SwerveModuleState backLeft;
        SwerveModuleState backRight;
        SwerveModule RFMod{1, 2, 9, true, RFZERO, false, false};
	    SwerveModule LFMod{8, 7, 12, true, LFZERO, false, false};
	    SwerveModule RBMod{3, 4, 10, false, RBZERO, false, false};
	    SwerveModule LBMod{5, 6, 11, true, LBZERO, false, false};
    
    private:
        frc::ADXRS450_Gyro gyro;
        //Relative Wheel positions to center(Correct Values Later)
        Translation2d frontLeftLocation{0.3175_m,0.3175_m};
        Translation2d frontRightLocation{0.3175_m, -0.3175_m};
        Translation2d backLeftLocation{-0.3175_m, 0.3175_m};
        Translation2d backRightLocation{-0.3175_m, -0.3175_m};

        SwerveDriveKinematics<4> kinematics{ frontLeftLocation, frontRightLocation,
            backLeftLocation, backRightLocation};
        //FIX ODOMETRY OBJECT
        SwerveDriveOdometry<4> odometry {kinematics, gyro.GetRotation2d()}; 

        Pose2d robotPose;
};