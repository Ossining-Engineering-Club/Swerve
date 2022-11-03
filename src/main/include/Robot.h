// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/MathUtil.h>
#include "SwerveModule.h"
#include "SwerveKnO.h"
#include <frc/TimedRobot.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/XboxController.h>

#include "Constants.h"
#include <frc/AnalogPotentiometer.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveModuleState.h>

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  double throttle;

 
 private:
  frc::SwerveModuleState zeroState;
  frc::Joystick stick1{0};
  frc::Joystick stick2{1};
  frc::XboxController controller{3};
  SmartDashboard* dash;
  SwerveModule RFMod;
	SwerveModule LFMod;
	SwerveModule RBMod;
	SwerveModule LBMod;
  ADXRS450_Gyro gyro;
  SwerveKnO KinematicsAndOdometry;
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  bool FieldOriented;
  //makes joystick input more gentel with a limit of 1/3 of a secont from 0 to 1
  frc::SlewRateLimiter<units::scalar> xspeedLimiter{30/1_s};
  frc::SlewRateLimiter<units::scalar> yspeedLimiter{30/1_s};
  frc::SlewRateLimiter<units::scalar> rotspeedLimiter{30/1_s};
  


};
