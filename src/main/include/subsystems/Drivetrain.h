/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Commands/Subsystem.h>
#include "frc/WPILib.h"
//#include "C:/Users/Admin/navx-mxp/cpp/include/AHRS.h"
#include "AHRS.h"



class Drivetrain : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

  frc::SpeedController    *leftMotor;
  frc::SpeedController    *rightMotor;
  frc::DifferentialDrive  *differentialDrive;


  frc::Encoder            *rightEncoder;
  frc::Encoder            *leftEncoder;

  

  AHRS *ahrs;	    //NavX

  frc::AnalogInput* analog0;
  frc::AnalogInput* analog1; 
  frc::AnalogInput* analog2; 

  unsigned char m_currLineState; //to make kris happy :)
  bool lineSensorsDeployed;

 public:

 const static int ENC_TICKS_PER_INCH;

  Drivetrain();
  void InitDefaultCommand() override;


  //*****Our Functions******

  //Encoders
	int  GetLeftEncoder(void);
	int  GetRightEncoder(void);
	void ResetEncoders(void);
  

  //Periodic
  void   DrivetrainPeriodic(void);

  //Drive
  void   DriveWithGamepad( void );
  void   Drive( double left, double right );
  void   Stop( void );

  //NavX
	bool   IsGyroConnected(void);
	double GetGyroYaw(void);            //yaw: Relative -180 to +180
	double GetGyroAngle(void);          //angle: absolute -inf to +inf
	double GetGyroRate(void);
	void   ZeroGyro(void);

  //Line Sensor/Follower
  bool LineFollower(void);
  void LineSensorsRetract(void);
  void LineSensorsDeploy(void);

};
