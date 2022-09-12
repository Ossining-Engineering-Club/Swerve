#pragma once
//offsets
#define RFZero 210.6
#define RBZero 215
#define LFZero 28.3
#define LBZero 312
//Gear and module related stuff
#define TurningMotorGearRatio 1/2 //find out
#define DrivingMotorGearRatio 1/2 //find out
#define WheelDiameterInMeters .123 //find out
#define DriveEncoderPosFactor (DrivingMotorGearRatio * M_PI * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60)
#define turnEncoderPosFactor (TurningMotorGearRatio * M_PI * 2)
#define turnEncoderVelocityFactor (turnEncoderPosFactor / 60)

#define MAXSPEED 1
#define MAXOmega 1

//PID for Drive
#define KDp 0
#define KDi 0
#define KDd 0
//PID for Rotate
#define KRp 1
#define KRi 0.0
#define KRd 0.0