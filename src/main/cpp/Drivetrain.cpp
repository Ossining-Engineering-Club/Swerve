#include "Drivetrain.h"

//KnO stands for Kinematics and Odometry
Drivetrain::Drivetrain(units::length::meter_t startingx, units::length::meter_t startingy, units::angle::radian_t  startingangle,units::angle::radian_t gyroAngle):
//Kinematics Object
kinematics(frontLeftLocation, frontRightLocation,backLeftLocation, backRightLocation),
//Odometry Object
odometry(kinematics, Rotation2d(gyroAngle), Pose2d(startingx, startingy,startingangle))
{
    //Chassis objects

}
//Columns are in order as frontleft, frontright, backleft, backright and the row is in order of speed and angle
void Drivetrain::FieldRelativeKinematics(
        units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed,
        units::radians_per_second_t angularVelocity,
        units::angle::radian_t CurrentAngle)
{
    //For Linear Motion
    /*gives Chassis object desired conditions so they can be evaluated via inverse kinematics to determine the necessary states of the modules (angles and speeds of each wheel in a matrix) */
    speeds = ChassisSpeeds::FromFieldRelativeSpeeds(xspeed,yspeed,angularVelocity, Rotation2d(CurrentAngle));
    //Invese kinematics to determine array of module states
    auto states = kinematics.ToSwerveModuleStates(speeds);
    auto [fl, fr, bl, br] = states;
    //referance t the address array
    kinematics.DesaturateWheelSpeeds(&states,(MAXMotorSPEED*1_mps));

    frontLeft = fl;
    frontRight = fr;
    backLeft = bl;
    backRight = br;
}
void Drivetrain::RobotRelativeKinematics(
        units::meters_per_second_t  xspeed,
        units::meters_per_second_t  yspeed,
        units::radians_per_second_t angularVelocity)
{
    speeds = ChassisSpeeds();
    speeds.omega = angularVelocity;
    speeds.vx = xspeed;
    speeds.vy = yspeed;
    //Invese kinematics to assign values to array of module states
    auto states = kinematics.ToSwerveModuleStates(speeds);
    auto [fl, fr, bl, br] = states;
    //referance t the address array
    kinematics.DesaturateWheelSpeeds(&states,(MAXMotorSPEED*1_mps));

    frontLeft = fl;
    frontRight = fr;
    backLeft = bl;
    backRight = br;
    /*Updates global matrix NOTE: fix later and make more elegant*/
}
void Drivetrain::SwerveOdometryGetPose(units::angle::radian_t gyroAngle)
{
    Rotation2d Angle = Rotation2d(gyroAngle);
    robotPose = odometry.Update(Angle, frontLeft,frontRight,backLeft,backRight);
}