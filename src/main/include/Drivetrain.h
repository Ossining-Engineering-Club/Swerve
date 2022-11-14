#pragma once
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "Constants.h"

using namespace frc;

//KnO stands for Kinematics and Odometry
class Drivetrain{
    public:
        Drivetrain(units::length::meter_t startingx,units::length::meter_t startingy, units::angle::radian_t startingangle,units::angle::radian_t gyroAngle);
        void FieldRelativeKinematics(units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed,
                                    units::radians_per_second_t angularVelocity,units::angle::radian_t CurrentAngle);
        void RobotRelativeKinematics(units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed, 
                                        units::radians_per_second_t angularVelocity);
        void SwerveOdometryGetPose(units::angle::radian_t gyroAngle);
        //States
        SwerveModuleState frontLeft;
        SwerveModuleState frontRight;
        SwerveModuleState backLeft;
        SwerveModuleState backRight;
        //Rows consist of motor data columns consist of motor unins ROWS: Meters per second, degrees
        //Vector for pose note that it is x, y, and rotation in degrees
        double PoseVector[3];
    
    private:
        SwerveDriveKinematics<4> kinematics;
        SwerveDriveOdometry<4> odometry;
        //Relative Wheel positions to center(Correct Values Later)
        Translation2d frontLeftLocation{0.3175_m,0.3175_m};
        Translation2d frontRightLocation{0.3175_m, -0.3175_m};
        Translation2d backLeftLocation{-0.3175_m, 0.3175_m};
        Translation2d backRightLocation{-0.3175_m, -0.3175_m};
        ChassisSpeeds speeds;
        Pose2d robotPose;
        Rotation2d Angle;

};