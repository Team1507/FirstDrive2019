/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "commands/GrpTest2.h"
#include <frc/WPILib.h>
#include "commands/CmdDriveClearAll.h"

OI::OI() {
  // Process operator interface input here.

   //Init Gamepads
  driver_gamepad   = new frc::Joystick(0); 
  

  frc::SmartDashboard::PutData("GrpTest2 Command", new GrpTest2() );
  frc::SmartDashboard::PutData("ZeroEncoder", new CmdDriveClearAll() );
}


//Public Gamepad Access
frc::Joystick* OI::DriverGamepad() {
   return driver_gamepad;
}