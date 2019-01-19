/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands\CmdDriveFwdEncoder.h"
#include "Robot.h"

CmdDriveFwdEncoder::CmdDriveFwdEncoder(double power, double distance, bool stop, double timeout)
{
  m_power    = power;
  m_distance = distance;
  m_stop     = stop;
  m_timeout  = timeout;

  // Use Requires() here to declare subsystem dependencies
  Requires( Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void CmdDriveFwdEncoder::Initialize() 
{
  Robot::m_drivetrain->ResetEncoders();

	if( m_timeout > 0.0)
	{
		SetTimeout (m_timeout);
	}

}

// Called repeatedly when this Command is scheduled to run
void CmdDriveFwdEncoder::Execute() 
{
 	double l_dir = Robot::m_drivetrain->GetLeftEncoder()/Drivetrain::ENC_TICKS_PER_INCH;
	double r_dir = Robot::m_drivetrain->GetRightEncoder()/Drivetrain::ENC_TICKS_PER_INCH;
	double delta = l_dir - r_dir;
	double kp    = -0.4; //was -0.18

	Robot::m_drivetrain->Drive(m_power - delta*kp  ,  m_power + delta*kp ); 
	//Robot::m_drivetrain->DriveAcc(m_power - delta*kp  ,  m_power + delta*kp ); 
}

// Make this return true when this Command no longer needs to run execute()
bool CmdDriveFwdEncoder::IsFinished() 
{

	double l_dir = Robot::m_drivetrain->GetLeftEncoder()/Drivetrain::ENC_TICKS_PER_INCH;
	double r_dir = Robot::m_drivetrain->GetRightEncoder()/Drivetrain::ENC_TICKS_PER_INCH;

	if(  (l_dir > m_distance) || (r_dir > m_distance)  )
	  return true;

	if ((m_timeout>0.0) && IsTimedOut())
	{
		std::cout<<"CmdDriveFwdEncoder: Timeout"<<std::endl;
		return true;
	}

	return false;
}

// Called once after isFinished returns true
void CmdDriveFwdEncoder::End() 
{
  if(m_stop)
	{
		Robot::m_drivetrain->Stop();
	}
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdDriveFwdEncoder::Interrupted() 
{
  End();
}
