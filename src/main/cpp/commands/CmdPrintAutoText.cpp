
#include "Commands\CmdPrintAutoText.h"


CmdPrintAutoText::CmdPrintAutoText(std::string text): frc::Command() 
{
    m_text = text;
}
  
// Called just before this Command runs the first time
void CmdPrintAutoText::Initialize() {
	std::cout << "*** Auto: "<< m_text<<std::endl;
}

// Called repeatedly when this Command is scheduled to run
void CmdPrintAutoText::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool CmdPrintAutoText::IsFinished() {return true;}

// Called once after isFinished returns true
void CmdPrintAutoText::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdPrintAutoText::Interrupted() {}
