/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class CmdDriveRevEncoder : public frc::Command {
 public:
    CmdDriveRevEncoder(double power, double distance, bool stop, double timeout);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;


  private:
    double m_power;
    double m_distance;
    bool   m_stop;
    double m_timeout;   

};
