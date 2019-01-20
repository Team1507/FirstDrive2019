/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CmdDeployLineSensors.h"

CmdDeployLineSensors::CmdDeployLineSensors(bool value)
{
  m_value = value;
}

// Called once when the command executes
void CmdDeployLineSensors::Initialize() 
{
  if(m_value)
    Robot::m_drivetrain->LineSensorsDeploy();
  else
    Robot::m_drivetrain->LineSensorsRetract();
}
