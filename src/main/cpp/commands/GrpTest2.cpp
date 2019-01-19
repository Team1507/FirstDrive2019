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

//#include "Commands/CmdLogEnable.h"
//#include "Commands/CmdLogMark.h"
//#include "Commands/CmdTurnPIDTest.h"

#include "Commands/CmdDriveClearAll.h"
#include "Commands/CmdDriveFwdEncoder.h"
#include "Commands/CmdDriveRevEncoder.h"
#include "Commands/CmdDriveTurn2Angle.h"
#include "Commands/CmdDriveTurn2Heading.h"
#define FWD_PWR 0.6
#define TRN_PWR 0.5

GrpTest2::GrpTest2() 
{
    AddSequential(new CmdPrintAutoText("GrpTest2 Begin"));
    AddSequential(new CmdDriveClearAll());
    AddSequential(new WaitCommand(0.25));
    //***************************************************
    AddSequential(new CmdDriveFwdGyro(0.4, 0, 120, true, 0.0)); //was 0.4,-70,80
    AddSequential(new CmdDriveTurn2Heading(0.45, -90)); //was 0.5,-90
    AddSequential(new CmdDriveFwdGyro(0.35, -90, 48, true, 0.0));
    //AddSequential(new CmdDriveFwdEncoder(0.4, 120, true, 0.0)); //was 0.4,120
    //AddSequential(new CmdDriveFwdEncoder(0.1, 5, true, 0.0)); //was 0.4,120
    //AddSequential(new CmdDriveRevEncoder(0.5, 60, true, 0.0));  //was 0.5, 60
   
  


    //***************************************************
    AddSequential(new WaitCommand(2.0));        //Let it finish whatever it's doing
    AddSequential(new CmdPrintAutoText("GrpTest2 Done!"));
}
