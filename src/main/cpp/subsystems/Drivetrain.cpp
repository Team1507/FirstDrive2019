/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "GamepadMap.h"
#include "Subsystems\Drivetrain.h"
#include "math.h"
#include "Commands\CmdDriveWithGamepad.h"
#include "C:/Users/Admin/navx-mxp/cpp/include/AHRS.h"

//Line Follower State Machine
#define STATE_LINE_HUNT 	 0
#define STATE_LINE_FOLLOW  	 1

#define BASE_THROTTLE      	.35  //.4	
#define THROTTLE_ADJUSTMENT .08
#define THROTTLE_MULTIPLIER	1.0

const int Drivetrain::ENC_TICKS_PER_INCH = 42;


Drivetrain::Drivetrain() : Subsystem("Drivetrain") 
{
	std::cout << "In Drivetrain" << std::endl;

	leftMotor         = new frc::Spark(0);
	rightMotor        = new frc::Spark(1);
	differentialDrive = new frc::DifferentialDrive(*leftMotor, *rightMotor);

	analog0 = new frc::AnalogInput(0);
  	analog1 = new frc::AnalogInput(1);
 	analog2 = new frc::AnalogInput(2);

	ahrs  	= new AHRS(SPI::Port::kMXP);

	//Disable Motor Saftey
	//Not a good idea, but required for GrpTest -> WaitCommand() 
	differentialDrive->SetSafetyEnabled(false);

	rightEncoder      = new frc::Encoder(0, 1, false, frc::Encoder::k4X);
	leftEncoder       = new frc::Encoder(2, 3, true, frc::Encoder::k4X);

	rightEncoder->SetDistancePerPulse(1.0);
	leftEncoder->SetDistancePerPulse(1.0);
	
}

void Drivetrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new CmdDriveWithGamepad() );
}

// Put methods for controlling this subsystem
// here. Call these from Commands.


//**************************************************************
int	Drivetrain::GetLeftEncoder(void)
{
	return leftEncoder->GetRaw();
}
int Drivetrain::GetRightEncoder(void)
{
	return rightEncoder->GetRaw();
}



//**************************************************************
void Drivetrain::ResetEncoders(void)
{
	leftEncoder->Reset();
	rightEncoder->Reset();
}

void Drivetrain::DrivetrainPeriodic(void)
{
	bool resetGyro = Robot::m_oi->DriverGamepad()->GetRawButtonPressed(GAMEPADMAP_BUTTON_X);
	if(resetGyro == true) 
	{
		ZeroGyro();
		std::cout << "Zero Gyro" << std::endl;
	}


	//****Line Follower Sensor ********************
	double voltage0 = analog0->GetVoltage() ;
  	double voltage1 = analog1->GetVoltage() ;
  	double voltage2 = analog2->GetVoltage() ;
  
	frc::SmartDashboard::PutNumber("v0", voltage0);
	frc::SmartDashboard::PutNumber("v1", voltage1);
	frc::SmartDashboard::PutNumber("v2", voltage2);

	bool leftEye;
	bool centerEye;
	bool rightEye;

	const double threshold = 3.2;
	
	if(voltage0 <= threshold) leftEye   = true; else (leftEye   = false);
	if(voltage1 <= threshold) centerEye = true; else (centerEye = false);
	if(voltage2 <= threshold) rightEye  = true; else (rightEye  = false);

	frc::SmartDashboard::PutBoolean("L", leftEye);
	frc::SmartDashboard::PutBoolean("C", centerEye);
	frc::SmartDashboard::PutBoolean("R", rightEye);
	frc::SmartDashboard::PutBoolean("Deployed", lineSensorsDeployed);
	
	int currState = 0;
	if(leftEye)  currState = currState + 100;
	if(centerEye)currState = currState + 10;
	if(rightEye) currState = currState + 1;
	m_currLineState = currState;

	//Write Gyro to dashboard
	frc::SmartDashboard::PutNumber("GyroAngle", GetGyroAngle());
	
}



//**************************************************************
void Drivetrain::DriveWithGamepad( void )
{
	const double deadband = 0.08;
	
	double yL = Robot::m_oi->DriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_Y);
	double xL = Robot::m_oi->DriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_X);
	double yR = Robot::m_oi->DriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_Y);
	double xR = -(Robot::m_oi->DriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_X)); //ben did this to fix L being R and R being L
	double tL = Robot::m_oi->DriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_TRIG);
	double tR = Robot::m_oi->DriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_TRIG);

	if (fabs(yL)<= deadband) yL = 0;
	if (fabs(xL)<= deadband) xL = 0;
	if (fabs(yR)<= deadband) yR = 0;
	if (fabs(xR)<= deadband) xR = 0;


	if(tR >= .5) //if Right Trigger pushed, Enable Line Follow if detected
	{
		//If Sensors not deployed, deploy them
		if( !lineSensorsDeployed )
			LineSensorsDeploy();

		//If LineFolower returns False (line not detected), use Gamepad for drive
		if ( !LineFollower() )
			differentialDrive->ArcadeDrive(yL,xR, true);

	}
	else 
	{
		//If sensors deployed, retract them
		if( lineSensorsDeployed )
			LineSensorsRetract();
		//Use Gamepad to drive
		differentialDrive->ArcadeDrive(yL,xR, true);
	}
		
	// 	//Arcade Drive
	//differentialDrive->ArcadeDrive(yL,xR,  true); THIS ONE IS GOOD HERE LOOK HERE

}

//**************************************************************
void Drivetrain::Drive( double left, double right )
{
	//Neg=Fwd.   Pos=Rev
	differentialDrive->TankDrive( (-1.0)*left,  (-1.0)*right,  false);
}
void Drivetrain::Stop( void )
{
	differentialDrive->TankDrive(0.0, 0.0, false);
  	std::cout << "STOP!" << std::endl;
}


//**************** AHRS (NavX) *********************
bool Drivetrain::IsGyroConnected(void)
{
	return ahrs->IsConnected();
}
double Drivetrain::GetGyroYaw(void)
{
    //Returns Relative Yaw:  -180 to +180
	return (double) ahrs->GetYaw();
}
double Drivetrain::GetGyroAngle(void)
{
    //returns total accumulated angle -inf to +inf  (continuous through 360deg)
	return (double) ahrs->GetAngle();
}
double Drivetrain::GetGyroRate(void)
{
	return ahrs->GetRate();
}
void Drivetrain::ZeroGyro(void)
{
  std::cout<<"ZeroGyro"<<std::endl;
	ahrs->ZeroYaw();
	//**OR
	//ahrs->Reset();//????
}


//**************** Line Follower *********************

bool Drivetrain::LineFollower(void)
{
	static unsigned char driveState = STATE_LINE_HUNT;
	double leftFollowThrottle;
	double rightFollowThrottle;

	switch(driveState)
	{
		
		case STATE_LINE_HUNT:
			if((m_currLineState > 0) && (m_currLineState < 111))//if the line is found
			{
				std::cout<<"ENTERING FOLLOW STATE"<<std::endl;
				driveState = STATE_LINE_FOLLOW; //follow it
			}
			return false;
			break;
		
		case STATE_LINE_FOLLOW:	
			if((m_currLineState == 0) || (m_currLineState == 111))//if the line lost/confused back to hunt
			{
				driveState = STATE_LINE_HUNT; //go back to hunt
				std::cout<<"Line lost, Leaving Follow."<<std::endl;
				return false;
			}
			else
			{
				frc::SmartDashboard::PutNumber("Current Line State", m_currLineState);
				//RUMBLE TIME BABYYYYYYYYY WOOOOOO
				//pretty rainbow lights baby woo (not as cool as rumble but whatever)
				switch(m_currLineState) //This is the switch for the actual line following
				{
					//THIS IS THE NIGHTMARE OF ALL THE CRAZY CASES. if 101...panic
					case 100:
						 leftFollowThrottle  = BASE_THROTTLE - THROTTLE_ADJUSTMENT; //was 2*throttle_adjust
						rightFollowThrottle  = BASE_THROTTLE + THROTTLE_ADJUSTMENT; //was 2*throttle_adjust
						break;
					case 110:
						leftFollowThrottle  =  BASE_THROTTLE; //- THROTTLE_ADJUSTMENT;
						rightFollowThrottle =  BASE_THROTTLE + THROTTLE_ADJUSTMENT;                  //SHOULD be a little less but we might not need to change it
						break;
					case 10:
						leftFollowThrottle = BASE_THROTTLE;
						rightFollowThrottle = BASE_THROTTLE;
						break;
					case 11:
						leftFollowThrottle  = BASE_THROTTLE + THROTTLE_ADJUSTMENT;
						rightFollowThrottle = BASE_THROTTLE; //- THROTTLE_ADJUSTMENT;               //SHOULD be a little less but we might not need to change it
						break;
					case 1:
						leftFollowThrottle  = BASE_THROTTLE + THROTTLE_ADJUSTMENT; //was 2*throttle_adjust
						rightFollowThrottle = BASE_THROTTLE - THROTTLE_ADJUSTMENT; //was 2*throttle_adjust
						break;
					case 101:
						driveState = STATE_LINE_HUNT;
						std::cout<<"OH MY GOODNESS PANIC 101 POTENTIAL SENSOR FAILURE BLAME BUILD TEAM"<<std::endl;
						return false;
						break;
					default:
						driveState = STATE_LINE_HUNT; //panic
						std::cout<<"oh my an invalid state oh no"<<std::endl;
						return false;
				}
				frc::SmartDashboard::PutNumber("LEFT THROTTLE", leftFollowThrottle);
				frc::SmartDashboard::PutNumber("RIGHT THROTTLE", rightFollowThrottle);
				Drive(leftFollowThrottle*(THROTTLE_MULTIPLIER), rightFollowThrottle*(THROTTLE_MULTIPLIER));	//was -1,-1
			}
			break;

		default:
			driveState = STATE_LINE_HUNT; //failsafe
			return false;
			break;

	}
	frc::SmartDashboard::PutNumber("LINE FOLLOW STATE", driveState);
	return true;

}

void Drivetrain::LineSensorsRetract(void)
{
	//RETRACT SENSORS HERE!!
	lineSensorsDeployed = false;
	std::cout << "Line Sensors Retract" << std::endl;
}
void Drivetrain::LineSensorsDeploy(void)
{
	//DEPLOY SENSORS HERE
	lineSensorsDeployed = true;
	std::cout << "Line Sensors Deploy" << std::endl;
}