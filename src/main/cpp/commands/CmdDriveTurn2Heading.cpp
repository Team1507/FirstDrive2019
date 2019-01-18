/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/CmdDriveTurn2Heading.h"
#include "Robot.h"

CmdDriveTurn2Heading::CmdDriveTurn2Heading(double power, double heading) 
{
    m_power   = power;
    m_heading = heading;

    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    Requires( Robot::m_drivetrain);

}

// Called just before this Command runs the first time
void CmdDriveTurn2Heading::Initialize() 
{
    m_isRightTurn = Robot::m_drivetrain->GetGyroYaw() < m_heading;

    if( m_isRightTurn)  std::cout<< "T2H: Right Turn" << std::endl;
    else                std::cout<< "T2H: Left Turn" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void CmdDriveTurn2Heading::Execute() 
{
	if( m_isRightTurn )
	{
		//Right turn.  Drive LEFT motor hard
		Robot::m_drivetrain->Drive(m_power, 0.1);
	}
	else
	{
		//Left turn.  Drive RIGHT motor hard
		Robot::m_drivetrain->Drive(0.1, m_power);
	}

}

// Make this return true when this Command no longer needs to run execute()
bool CmdDriveTurn2Heading::IsFinished()
{ 
	if( m_isRightTurn )
	{
		//Right turn: Done when Yaw becomes greater than calc angle
		if(Robot::m_drivetrain->GetGyroYaw() >= m_heading)
		{
			return true;
		}
	}
	else
	{
		//Left Turn:  Done when Yaw becomes less than calc angle
		if(Robot::m_drivetrain->GetGyroYaw() <= m_heading)
		{
			return true;
		}
	}
	return false;
}

// Called once after isFinished returns true
void CmdDriveTurn2Heading::End() 
{
    Robot::m_drivetrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdDriveTurn2Heading::Interrupted() {}
