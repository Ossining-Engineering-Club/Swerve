#include "Drivetrain.h"

//Initialize with starting position and angle on field
Drivetrain::Drivetrain(units::length::meter_t startingx, units::length::meter_t startingy, units::angle::radian_t  startingangle):

//Initialize Odometry
odometry(kinematics, Rotation2d(startingangle), Pose2d(startingx, startingy,startingangle))
{
    //Chassis objects

}
void Drivetrain::Drive(
        units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed,
        units::radians_per_second_t angularVelocity,
        bool FieldOreinted)
{
  auto states = kinematics.ToSwerveModuleStates(
      FieldOreinted ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                        xspeed, yspeed, angularVelocity, 
                        gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xspeed, yspeed, 
                        angularVelocity});

  kinematics.DesaturateWheelSpeeds(&states, (MAX_SPEED*1_mps));

  auto [fl, fr, bl, br] = states;

  LFMod.SetToVector(fl);
  RFMod.SetToVector(fr);
  LBMod.SetToVector(bl);
  RBMod.SetToVector(br);
}
void Drivetrain::UpdateOdometry() { 
    odometry.Update(gyro.GetRotation2d(), LFMod.GetState(),
    RFMod.GetState(), LBMod.GetState(), RBMod.GetState());
}
void Drivetrain::SwerveOdometryGetPose(units::angle::radian_t gyroAngle)
{
    Rotation2d Angle = Rotation2d(gyroAngle);
    robotPose = odometry.Update(Angle, frontLeft,frontRight,backLeft,backRight);
}
void Drivetrain::ResetDrive()
{
    //Important: If at any time, you reset your gyroscope, 
    // the resetPose method MUST be called with the new gyro angle.
    //The robot pose can be reset via the resetPose method. This method 
    //accepts two arguments – the new field-relative pose 
    //and the current gyro angle.

    gyro.Reset();
    LFMod.ResetEncoder(); LBMod.ResetEncoder();
    RFMod.ResetEncoder(); RBMod.ResetEncoder();
}
double Drivetrain::GetGyro()
{
    return (gyro.GetAngle()-90.0)*(M_PI/180.0);
}
double Drivetrain::GetValue(int swerve_module, int readItem)
{
    double readout=0.0;
    switch (readItem) {
        case ABS_ANGLE:
            if(swerve_module == L_FRONT) readout=LFMod.GetAbsEncoderAngle();
            else if(swerve_module == R_FRONT) readout=RFMod.GetAbsEncoderAngle();
            else if(swerve_module == L_BACK) readout=LBMod.GetAbsEncoderAngle();
            else if(swerve_module == R_BACK) readout=RBMod.GetAbsEncoderAngle();
            break;            
    }
    return (readout);
}