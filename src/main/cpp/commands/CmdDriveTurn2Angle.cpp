/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/CmdDriveTurn2Angle.h"
#include "Robot.h"


CmdDriveTurn2Angle::CmdDriveTurn2Angle(double power, double angle) 
{
    m_power = power;
    m_angle = angle;

    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    Requires( Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void CmdDriveTurn2Angle::Initialize() 
{
	m_calcAngle   = Robot::m_drivetrain->GetGyroAngle() + m_angle;	//Calculate degrees to turn
	m_isRightTurn = m_angle > 0;								    //Right turn if angle positive    
}

// Called repeatedly when this Command is scheduled to run
void CmdDriveTurn2Angle::Execute() 
{
	if( m_isRightTurn )
	{
		//Right turn.  Drive LEFT motor hard
		Robot::m_drivetrain->Drive(m_power, -0.1);
	}
	else
	{
		//Left turn.  Drive RIGHT motor hard
		Robot::m_drivetrain->Drive(-0.1, m_power);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool CmdDriveTurn2Angle::IsFinished() 
{
	if( m_isRightTurn )
	{
		//Right turn: Done when Yaw becomes greater than calc angle
		if(Robot::m_drivetrain->GetGyroAngle() >= m_calcAngle)
		{
			return true;
		}
	}
	else
	{
		//Left Turn:  Done when Yaw becomes less than calc angle
		if(Robot::m_drivetrain->GetGyroAngle() <= m_calcAngle)
		{
			return true;
		}
	}
	return false; 
}

// Called once after isFinished returns true
void CmdDriveTurn2Angle::End() 
{
	Robot::m_drivetrain->Stop();    
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdDriveTurn2Angle::Interrupted() {}
