#pragma once
//offsets
#define RFZero 2.960583
#define RBZero 1.501767
#define LFZero 0.771592
#define LBZero 5.220137
//Gear and module related stuff
#define TurningMotorGearRatio 1/(150/7) //find out
#define DrivingMotorGearRatio 1/6.75 //find out
#define WheelDiameterInMeters 0.1016 //find out
#define DriveEncoderPosFactor (DrivingMotorGearRatio * M_PI * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60)
#define turnEncoderPosFactor (TurningMotorGearRatio * M_PI * 2)
#define turnEncoderVelocityFactor (turnEncoderPosFactor / 60)

#define MAXMotorSPEED 4.4196 //in meters per second converted from ft/s
#define MAXTotalSPEED 4.4196/4 
#define MAXOmega 4//Calculate after taking dimensions

//PID for Drive
#define KDp 0
#define KDi 0
#define KDd 0
//PID for Rotate
#define KRp 1
#define KRi 0.0
#define KRd 0.0