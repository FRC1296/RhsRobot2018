/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <string>
#include <thread>

//Robot
#include "WPILib.h"
#include "ComponentBase.h" //For the ComponentBase class
#include "RobotParams.h" //For various robot parameters

//#define TEST_SCRIPTS

// if you have more than this many lines in your script, THEY WILL NOT RUN! Change if needed.
const char* const AUTONOMOUS_SCRIPT_FILEPATH = "/home/lvuser/RhsScript.txt";

class Autonomous : public ComponentBase
{
public:
	Autonomous();
	~Autonomous();
	void DoScript();

	static void *StartTask(void *pThis)
	{
		((Autonomous *)pThis)->DoWork();
		return(NULL);
	}

	static void *StartScript(void *pThis)
	{
		((Autonomous *)pThis)->DoScript();
		return(NULL);
	}

protected:
	bool Evaluate(std::string statement);	//Evaluates an autonomous script statement
	RobotMessage Message;
	bool bScriptLoaded; //not yet in use
	bool bInAutoMode;
	bool bPauseAutoMode;

private:
	std::vector<std::string> script;
	int lineNumber;
	int iAutoDebugMode;
	std::thread *pScript;
	bool bReceivedCommandResponse;
	unsigned int uResponseCount;
	MessageCommand ReceivedCommand;
	Timer *pDebugTimer;
	char szModeString[8];
	bool bModeFound;

	bool Begin(char *);
	bool End(char *);
	void Delay(float);
	bool Move(char *);
	bool MeasuredMove(char *);
	bool VelocityMove(char *);
	bool Turn(char *);
	bool Elevator(char *);
	bool Claw(char *);
	bool Arm(char *);
	bool SPunch(char *);

	bool CommandResponse(const char *szQueueName);
	bool CommandNoResponse(const char *szQueueName);
	bool MultiCommandResponse(vector<char*> szQueueNames, vector<MessageCommand> commands);

	void Init();
	void OnStateChange();
	void Run();
	bool LoadScriptFile();
};

#endif //AUTONOMOUS_BASE_H
