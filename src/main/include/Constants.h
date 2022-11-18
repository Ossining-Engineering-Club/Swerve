#pragma once
//offsets
#define RFZero 4.519107
#define RBZero 3.092505
#define LFZero -0.766990
#define LBZero -0.510816
//Gear and module related stuff
#define TurningMotorGearRatio 7.0/150.0 //find out
#define DrivingMotorGearRatio 1/6.75 //find out
#define WheelDiameterInMeters 0.1016 //find out
#define DriveEncoderPosFactor (DrivingMotorGearRatio * M_PI * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60)
#define turnEncoderPosFactor (TurningMotorGearRatio * M_PI *2)
#define turnEncoderVelocityFactor (turnEncoderPosFactor / 60)

#define MAXMotorSPEED 4.4196 //in meters per second converted from ft/s
#define MAXTotalSPEED 4.4196/4 
#define MAXOmega 1//Calculate after taking dimensions

//PID for Drive
#define KDp 0
#define KDi 0
#define KDd 0
//PID for Rotate
#define KRp 1
#define KRi 0.0
#define KRd 0.0