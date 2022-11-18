
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string>
#include <frc/MathUtil.h>
#include "Drivetrain.h"
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "Constants.h"
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/kinematics/SwerveModuleState.h>

class Robot : public frc::TimedRobot {
  private:
  frc::XboxController controller{3};
  frc::SmartDashboard* dash;
  Drivetrain swerve;
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::Field2d m_field;
  bool FieldOriented;
  //Limit joystick input to 1/3 of a secont from 0 to 1
  frc::SlewRateLimiter<units::scalar> xspeedLimiter{30/1_s};
  frc::SlewRateLimiter<units::scalar> yspeedLimiter{30/1_s};
  frc::SlewRateLimiter<units::scalar> rotspeedLimiter{30/1_s};

public: 
  Robot():
      //Initializa swerve_drive with the starting position and angle on the field
      swerve(5_m,5_m,0.0*1_rad)
      { 
        dash -> init();
      }

  void RobotInit() {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    frc::SmartDashboard::PutData("Field",&m_field);
  }

  void AutonomousInit() {
    FieldOriented = FIELD_ORIENTED; // Use Robot Oriented Control for Auto
    swerve.ResetDrive();
  }

void AutonomousPeriodic() {
      swerve.UpdateOdometry();
  }

  void TeleopInit() {
    // For actual game, we will want to line up robot square
    // Then reset gyro in Auto and not in Teleop
    FieldOriented = FIELD_ORIENTED; 
    swerve.ResetDrive();
  
  }

  void TeleopPeriodic() {
    m_field.SetRobotPose(swerve.SwerveOdometryGetPose());
    //Note: Xbox inputs are reversed compared to joystick
    //Note: Deadband in Xbox controller
    //Add exponential limmiter or reuse # of increments of circle
    const auto xSpeed = xspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(),0.4)*MAX_SPEED);
    const auto ySpeed = yspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetRightY(),0.4)*MAX_SPEED);
    const auto rotSpeed = rotspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(),0.4)*MAX_TURN_RATE);
    
    //swerve_drive.SwerveOdometryGetPose(gyro.GetAngle()*1_rad);
    swerve.Drive((xSpeed*1_mps),(ySpeed*1_mps),(rotSpeed*1_rad_per_s),FieldOriented);
    swerve.UpdateOdometry();

    //Dash should be updated 1 out of every 100 calls to not slow things down
<<<<<<< Updated upstream
=======
    // Here is an alteraitnve read if the current format fails
    //dash->PutNumber("ABSLFPos",swerve.GetValue(L_FRONT, ABS_ANGLE));
    /*
>>>>>>> Stashed changes
    dash->PutNumber("ABSLBPos",swerve.LFMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSLBPos",swerve.LBMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSRFPos",swerve.RFMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSRBPos",swerve.RBMod.GetAbsEncoderAngle());
    dash->PutNumber("LFPos",swerve.LFMod.GetCurrentAngle());
    dash->PutNumber("LBPos",swerve.LBMod.GetCurrentAngle());
    dash->PutNumber("RFPos",swerve.RFMod.GetCurrentAngle());
    dash->PutNumber("RBPos",swerve.RBMod.GetCurrentAngle());
    dash->PutNumber("RLFPos",swerve.LFMod.GetRotatorPower());
    dash->PutNumber("RLBPos",swerve.LBMod.GetRotatorPower());
    dash->PutNumber("RRFPos",swerve.RFMod.GetRotatorPower());
    dash->PutNumber("RRBPos",swerve.RBMod.GetRotatorPower()); 
    dash->PutNumber("DLFPos",swerve.LFMod.GetDrivePower());
    dash->PutNumber("DLBPos",swerve.LBMod.GetDrivePower());
    dash->PutNumber("DRFPos",swerve.RFMod.GetDrivePower());
    dash->PutNumber("DRBPos",swerve.RBMod.GetDrivePower());
    dash->PutNumber("Gyro", swerve.GetGyro());*/
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
