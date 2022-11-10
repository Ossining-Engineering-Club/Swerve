
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string>
#include <frc/MathUtil.h>
#include "SwerveModule.h"
#include "Drivetrain.h"
#include <frc/TimedRobot.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/XboxController.h>
#include "Constants.h"
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveModuleState.h>

class Robot : public frc::TimedRobot {
 
 private:
  frc::SwerveModuleState zeroState;
  frc::XboxController controller{3};
  SmartDashboard* dash;
  SwerveModule RFMod{1, 2, 9, true,(RFZERO),true,false};
	SwerveModule LFMod{8, 7, 12, true,(LFZERO),false,false};
	SwerveModule RBMod{3, 4, 10, false,(RBZERO),false,false};
	SwerveModule LBMod{5, 6, 11, true,(LBZERO),false,false};
  ADXRS450_Gyro gyro;
  Drivetrain swerve_drive;
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  bool FieldOriented;
  //makes joystick input more gentel with a limit of 1/3 of a secont from 0 to 1
  frc::SlewRateLimiter<units::scalar> xspeedLimiter{30/1_s};
  frc::SlewRateLimiter<units::scalar> yspeedLimiter{30/1_s};
  frc::SlewRateLimiter<units::scalar> rotspeedLimiter{30/1_s};

public: 
  Robot():
      gyro(),
      swerve_drive(0_m,0_m,0_rad, (gyro.GetAngle()*1_rad))
      { 
        dash -> init();
      }

  void RobotInit() {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  }

  void AutonomousInit() {
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    fmt::print("Auto selected: {}\n", m_autoSelected);
    if (m_autoSelected == kAutoNameCustom) {
      // Custom Auto goes here
    } else {
      // Default Auto goes here
    }
  }

  void TeleopInit() {
    gyro.Reset();
    FieldOriented = true;
    LFMod.ResetEncoder();
    LBMod.ResetEncoder();
    RFMod.ResetEncoder();
    RBMod.ResetEncoder();
  }

  void TeleopPeriodic() {
    //Note: Xbox inputs are reversed compared to joystick
    //Note: Deadband in Xbox controller
    //Add exponential limmiter or reuse # of increments of circle
    const auto xSpeed = xspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(),0.4)*MAXMotorSPEED);
    const auto ySpeed = yspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetRightY(),0.4)*MAXMotorSPEED);
    const auto rotSpeed = rotspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(),0.4)*MAXOmega);
    
    //swerve_drive.SwerveOdometryGetPose(gyro.GetAngle()*1_rad);
    if(FieldOriented) swerve_drive.FieldRelativeKinematics((xSpeed*1_mps),(ySpeed*1_mps), 
                          (rotSpeed*1_rad_per_s),((gyro.GetAngle()-90)*(M_PI/180)*1_rad));
    else swerve_drive.RobotRelativeKinematics((0.5_mps),(0.5_mps),(0_rad_per_s));
 
    dash->PutNumber("ABSLFPos",LFMod.GetAbsEncoderPosition());
    dash->PutNumber("ABSLBPos",LBMod.GetAbsEncoderPosition());
    dash->PutNumber("ABSRFPos",RFMod.GetAbsEncoderPosition());
    dash->PutNumber("ABSRBPos",RBMod.GetAbsEncoderPosition());
    dash->PutNumber("LFPos",LFMod.GetCurrentPosition());
    dash->PutNumber("LBPos",LBMod.GetCurrentPosition());
    dash->PutNumber("RFPos",RFMod.GetCurrentPosition());
    dash->PutNumber("RBPos",RBMod.GetCurrentPosition());
    dash->PutNumber("RLFPos",LFMod.GetRotatorPower(swerve_drive.frontLeft));
    dash->PutNumber("RLBPos",LBMod.GetRotatorPower(swerve_drive.backLeft));
    dash->PutNumber("RRFPos",RFMod.GetRotatorPower(swerve_drive.frontRight));
    dash->PutNumber("RRBPos",RBMod.GetRotatorPower(swerve_drive.backRight)); 
    dash->PutNumber("DLFPos",LFMod.GetDrivePower(swerve_drive.frontLeft));
    dash->PutNumber("DLBPos",LBMod.GetDrivePower(swerve_drive.backLeft));
    dash->PutNumber("DRFPos",RFMod.GetDrivePower(swerve_drive.frontRight));
    dash->PutNumber("DRBPos",RBMod.GetDrivePower(swerve_drive.backRight));
    dash->PutNumber("Gyro", gyro.GetAngle()*(M_PI/180));
    LFMod.SetToVector(swerve_drive.frontLeft);
    RFMod.SetToVector(swerve_drive.frontRight);
    LBMod.SetToVector(swerve_drive.backLeft);
    RBMod.SetToVector(swerve_drive.backRight);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
