/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands\CmdDriveLineDriftLeft.h"
#include "Robot.h"

CmdDriveLineDriftLeft::CmdDriveLineDriftLeft(double power, double heading, double distance, bool stop, double timeout)
{
  m_power    = power;
	m_heading  = heading - 3.0;		//Force left drift to intercept tape line
  m_distance = distance;
  m_stop     = stop;
  m_timeout  = timeout;

  // Use Requires() here to declare subsystem dependencies
  Requires( Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void CmdDriveLineDriftLeft::Initialize() 
{
  Robot::m_drivetrain->ResetEncoders();

	if( m_timeout > 0.0)
	{
		SetTimeout (m_timeout);
	}

}

// Called repeatedly when this Command is scheduled to run
void CmdDriveLineDriftLeft::Execute() 
{
	//double errorAngle = Robot::m_drivetrain->GetGyroYaw() - m_heading;
	double errorAngle = Robot::m_drivetrain->GetGyroAngle() - m_heading;
	double kp = 0.04; //was 0.05

	//Run Follow Line Algorithm.
	//If line found, returns true and we do nothing
	//If line not found, returns false and we have to drive
	if( !Robot::m_drivetrain->LineFollower() )
		Robot::m_drivetrain->Drive(m_power - errorAngle*kp  ,  m_power + errorAngle*kp ); 
	
}

// Make this return true when this Command no longer needs to run execute()
bool CmdDriveLineDriftLeft::IsFinished() 
{

	double l_dir = Robot::m_drivetrain->GetLeftEncoder()/Drivetrain::ENC_TICKS_PER_INCH;
	double r_dir = Robot::m_drivetrain->GetRightEncoder()/Drivetrain::ENC_TICKS_PER_INCH;

	if(  (l_dir > m_distance) || (r_dir > m_distance)  )
	  return true;

	if ((m_timeout>0.0) && IsTimedOut())
	{
		std::cout<<"CmdDriveFwdGyro: Timeout"<<std::endl;
		return true;
	}

	return false;
}

// Called once after isFinished returns true
void CmdDriveLineDriftLeft::End() 
{
  if(m_stop)
	{
		std::cout<<"GyroAngle ";
		std::cout<<Robot::m_drivetrain->GetGyroAngle()<<std::endl;
		std::cout<<"EncoderValueL ";
		std::cout<<Robot::m_drivetrain->GetLeftEncoder()<<std::endl;
		std::cout<<"EncoderValueR ";
		std::cout<<Robot::m_drivetrain->GetRightEncoder()<<std::endl;
		Robot::m_drivetrain->Stop();
	}
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdDriveLineDriftLeft::Interrupted() 
{
  End();
}
