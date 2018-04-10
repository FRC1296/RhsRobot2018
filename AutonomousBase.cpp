/** \file
 * The AutonomousBase component class handles basic autonomous functionality.
 */

#include <iostream>
#include <fstream>
#include <string>

//Local

#include "ComponentBase.h"
#include "Autonomous.h"
#include "RobotParams.h"
#include "WPILib.h"


using namespace std;

Autonomous::Autonomous()
: ComponentBase(AUTONOMOUS_TASKNAME, AUTONOMOUS_QUEUE, AUTONOMOUS_PRIORITY)
{
	lineNumber = 0;
	bInAutoMode = false;
	iAutoDebugMode = 0;
	uResponseCount = 0;
	bReceivedCommandResponse = false;
	ReceivedCommand = COMMAND_UNKNOWN;

	bPauseAutoMode = false;
	bScriptLoaded = false;
	writingString = false;

	pDebugTimer = new Timer();
	pDebugTimer->Start();

	szModeString[0] = 0;
	bModeFound = false;

	pTask = new std::thread(&Autonomous::StartTask, this);
	wpi_assert(pTask);

	pScript = new std::thread(&Autonomous::StartScript, this);
	wpi_assert(pScript);
}

Autonomous::~Autonomous()	//Destructor
{
	delete(pTask);
	delete(pScript);
}

void Autonomous::Init()	//Initializes the autonomous component
{
}

void Autonomous::OnStateChange()	//Handles state changes
{
	// to handle unexpected state changes before the auto script finishes (like in OKC last year)
	// we will leave the script running

	if(localMessage.command == COMMAND_ROBOT_STATE_AUTONOMOUS)
	{
		bPauseAutoMode = false;
		bInAutoMode = true;
		pDebugTimer->Reset();
	}
	else if(localMessage.command == COMMAND_ROBOT_STATE_TELEOPERATED)
	{
		bPauseAutoMode = true;
		bInAutoMode = false;
		printf("teleop state recieved - auto");
	}
	else if(localMessage.command == COMMAND_ROBOT_STATE_DISABLED)
	{
		bPauseAutoMode = true;
		bInAutoMode = false;

		printf("disabled state recieved - auto");
	}
}

void Autonomous::Run()
{
	switch(localMessage.command)
	{
	case COMMAND_AUTONOMOUS_RUN:
		break;

	case COMMAND_CHECKLIST_RUN:
		break;

	case COMMAND_AUTONOMOUS_RESPONSE_OK:
		uResponseCount++;
		bReceivedCommandResponse = true;
		ReceivedCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
		break;

	case COMMAND_AUTONOMOUS_RESPONSE_ERROR:
		uResponseCount++;
		bReceivedCommandResponse = true;
		ReceivedCommand = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
		break;

	case COMMAND_SYSTEM_GAMEDATA:

		writingString = true;
		if(localMessage.params.gamedata.eSwitchSide == GAMEPIECESIDE_LEFT)
		{
			szModeString[1] = 'L';
		}
		else
		{
			szModeString[1] = 'R';
		}

		if(localMessage.params.gamedata.eScaleSide == GAMEPIECESIDE_LEFT)
		{
			szModeString[2] = 'L';
		}
		else
		{
			szModeString[2] = 'R';
		}

		if(localMessage.params.gamedata.eOpponentSwitchSide == GAMEPIECESIDE_LEFT)
		{
			szModeString[3] = 'L';
		}
		else
		{
			szModeString[3] = 'R';
		}

		if(localMessage.params.gamedata.eStartingPosition == GAMEPIECESTART_LEFT)
		{
			szModeString[0] = 'L';
		}
		else if(localMessage.params.gamedata.eStartingPosition == GAMEPIECESTART_CENTER)
		{
			szModeString[0] = 'C';
		}
		else if(localMessage.params.gamedata.eStartingPosition == GAMEPIECESTART_RIGHT)
		{
			szModeString[0] = 'R';
		}
		else
		{
			szModeString[0] = 'X';
		}

		szModeString[4] = 0;
		writingString = false;

		if(strncmp(szModeString,szPrevModeString, 4))
		{
			SmartDashboard::PutString("AutoModeStrng", szModeString);
			printf("auto mode change %s\n", szModeString);

		}
		else
		{
			SmartDashboard::PutString("AutoModeStrng", szModeString);
			printf("auto mode same %s\n", szModeString);
		}

		szPrevModeString[0] = szModeString[0];
		szPrevModeString[1] = szModeString[1];
		szPrevModeString[2] = szModeString[2];
		szPrevModeString[3] = szModeString[3];
		szPrevModeString[4] = szModeString[4];
		break;

	default:
		break;
	}
}

bool Autonomous::LoadScriptFile()
{
	bool bReturn = true;
	printf("Auto Script Filepath: [%s]\n", AUTONOMOUS_SCRIPT_FILEPATH);
	ifstream scriptStream;
	scriptStream.open(AUTONOMOUS_SCRIPT_FILEPATH, ios::in);
	string newLine;

	if(scriptStream.is_open())//not working
	{
		while(getline(scriptStream, newLine))
		{
			cout << newLine << endl;
			script.push_back(newLine);
		}

		printf("Autonomous script loaded\n");
		scriptStream.close();
	}
	else
	{
		printf("No auto file found\n");
		bReturn = false;
	}

	return(bReturn);
}

void Autonomous::DoScript()
{
	//int loadAttemptTally = 0; //for debugging
	SmartDashboard::PutString("Script Line", "DoScript started");
	SmartDashboard::PutString("Auto Status", "Ready to go");
	SmartDashboard::PutBoolean("Script File Loaded", false);
	printf("DoScript\n");

	// executes autos over and over again

	while(true)
	{
		if(LoadScriptFile() == false)
		{
			// wait a little and try again, really only useful if when practicing

			SmartDashboard::PutBoolean("Script File Not Found", false);
			Wait(1.0);
		}
		else
		{
			SmartDashboard::PutBoolean("Script File Found", true);
			break;
		}
	}

	// if there is a script we will execute it some heck or high water!
	while (!bInAutoMode)
	{
		Wait(0.1);
	}

	while (bInAutoMode)
	{
		std::vector<std::string>::iterator nextLine;

		if (!bPauseAutoMode)
		{
			for(nextLine = script.begin(); nextLine != script.end(); ++nextLine)
			{
				// handle pausing in the Evaluate method

				if (Evaluate(*nextLine) == true)
				{
					// error executing this line or end of script
					SmartDashboard::PutString("Script Line", "<NOT RUNNING>");
					bInAutoMode = false;
					return;
					//break;
				}
				if(!bInAutoMode)
					break;
			}
		}
	}

	bInAutoMode = false;
}




