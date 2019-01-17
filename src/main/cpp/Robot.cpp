/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include "AHRS.h"
ExampleSubsystem Robot::m_subsystem;
OI *Robot::m_oi;
Drivetrain *Robot::m_drivetrain;



void Robot::RobotInit() 
{
  m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
  m_chooser.AddOption("My Auto", &m_myAuto);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	//******** INIT *************************************
  std::cout<<"JustDrive"<<std::endl;
  std::cout<<"Deep Space 2019"<<std::endl;

	//Display Version info to Log file, for posterity
	std::cout<<"Version: " << __DATE__ <<"  "<<__TIME__<<std::endl<<std::endl;

  //Put all out stuff here
  //Subsystem Inits
  m_drivetrain = new Drivetrain();

  //OI **MUST** be after all subsystem Inits
  m_oi = new OI();

  ahrs = new AHRS(SPI::Port::kMXP);


  //CAMERA
  // 	//the driver camera, to be fixed
	
  
  //cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

	// camera.SetResolution(160, 120);

  // //camera.SetVideoMode(cs::VideoMode::kGray, 320, 240, 8);
	// camera.SetFPS(15);
  // camera.SetBrightness(20);       //100-most bright //0- most dark
  // camera.SetWhiteBalanceManual(cs::VideoCamera::kFixedFluorescent1);
  // camera.SetExposureManual(38);   //0- Mbps 0.73, darker //100- Mbps .31, lighter


}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  m_drivetrain->DrivetrainPeriodic();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() 
{
  std::cout<<"DisabledInit"<<std::endl;

}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
  // std::string autoSelected = frc::SmartDashboard::GetString(
  //     "Auto Selector", "Default");
  // if (autoSelected == "My Auto") {
  //   m_autonomousCommand = &m_myAuto;
  // } else {
  //   m_autonomousCommand = &m_defaultAuto;
  // }

  m_autonomousCommand = m_chooser.GetSelected();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Start();
  }

   std::cout<<"AutoInit"<<std::endl; 

}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

   std::cout<<"TeleopInit"<<std::endl; 
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
