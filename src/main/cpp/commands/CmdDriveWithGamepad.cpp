/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Commands\CmdDriveWithGamepad.h"

CmdDriveWithGamepad::CmdDriveWithGamepad() {
  // Use Requires() here to declare subsystem dependencies
  Requires( Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void CmdDriveWithGamepad::Initialize() 
{
  std::cout<<"CmdDriveWithGamepad Started" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void CmdDriveWithGamepad::Execute() 
{
  Robot::m_drivetrain->DriveWithGamepad();
}

// Make this return true when this Command no longer needs to run execute()
bool CmdDriveWithGamepad::IsFinished() { return false; }

// Called once after isFinished returns true
void CmdDriveWithGamepad::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdDriveWithGamepad::Interrupted() 
{
  std::cout<<"CmdDriveWithGamepad Interrupted" << std::endl;
}
