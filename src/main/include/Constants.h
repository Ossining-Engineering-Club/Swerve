#pragma once
//offsets
#define RFZERO 1.761010
#define RBZERO 3.204486
#define LFZERO 5.514661
#define LBZERO 5.787710
//Gear and module related stuff
#define TurningMotorGearRatio 7.0/150.0 //find out
#define DrivingMotorGearRatio 1/6.75 //find out
#define WheelDiameterInMeters 0.1016 //find out
#define DriveEncoderPosFactor (DrivingMotorGearRatio * M_PI * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60)
#define turnEncoderPosFactor (TurningMotorGearRatio * M_PI *2)
#define turnEncoderVelocityFactor (turnEncoderPosFactor / 60)
#define MAXMotorSPEED 4.4196 //in meters per sec 
#define MAXOmega 1.0 //Calculate after taking dimensions
//PID for Drive
#define KDp 1.0
#define KDi 0.0
#define KDd 0.0
//PID for Rotate
#define KRp 1.0
#define KRi 0.0
#define KRd 0.0