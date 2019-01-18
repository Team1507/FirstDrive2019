#ifndef CMDPRINTAUTOTEXT_H
#define CMDPRINTAUTOTEXT_H


#include "frc/commands/Subsystem.h"
#include "Robot.h"

class CmdPrintAutoText: public frc::Command {
public:
	CmdPrintAutoText(std::string text);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
    std::string m_text;

};

#endif
