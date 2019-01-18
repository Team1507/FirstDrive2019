/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/GrpTest2.h"
#include "Commands/CmdPrintAutoText.h"
#include "Commands/CmdDriveFwdEncoder.h"
#include "Commands/CmdDriveRevEncoder.h"
#include "Commands/CmdDriveFwdGyro.h"
#include "Commands/CmdDriveRevGyro.h"
//#include "Commands/CmdDriveTurn2Heading.h"
//#include "Commands/CmdDriveTurn2Angle.h"

//#include "Commands/CmdLogEnable.h"
//#include "Commands/CmdLogMark.h"

#include "Commands/CmdDriveClearAll.h"
//#include "Commands/CmdTurnPIDTest.h"
#include "Commands/CmdDriveFwdEncoder.h"
#include "Commands/CmdDriveRevEncoder.h"
#include "Commands/CmdDriveTurn2Angle.h"

#define FWD_PWR 0.6
#define TRN_PWR 0.5

GrpTest2::GrpTest2() 
{
    AddSequential(new CmdPrintAutoText("GrpTest2 Begin"));
    AddSequential(new CmdDriveClearAll());
    AddSequential(new WaitCommand(0.25));
    //***************************************************
    AddSequential(new CmdDriveFwdEncoder(0.4, 120, true, 0.0)); 
    //AddSequential(new CmdDriveRevEncoder(0.5, 60, true, 0.0)); 
    AddSequential(new CmdDriveTurn2Angle(0.4, -70)); 
    AddSequential(new CmdDriveFwdGyro(0.4, -70, 80, true, 0.0)); 
  


    //***************************************************
    AddSequential(new WaitCommand(2.0));        //Let it finish whatever it's doing
    AddSequential(new CmdPrintAutoText("GrpTest2 Done!"));
}
