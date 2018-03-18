/** \file
 *  Autonomous script parser
 */

#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>

#include <WPILib.h>

//Robot
#include "Autonomous.h"
#include "AutoParser.h"
#include "ComponentBase.h"
#include "RobotParams.h"

using namespace std;



const char *szTokens[] = {
		"MODE",
		"DEBUG",
		"MESSAGE",
		"BEGIN",
		"END",
		"DELAY",			//!<(seconds)
		"MOVE",				//!<(left speed) (right speed)
		"MMOVE",	        //!<(speed) (distance:inches) (timeout)
		"VMOVE",			//!<(speed) (distance:inches) (timeout)
		"TURN",				//!<(degrees) (timeout)
		"ELEVATOR",
		"CLAW",
		"ARM",
		"PUNCH",
		"NOP" };

bool Autonomous::Evaluate(std::string rStatement) {
	char *pToken;
	char *pCurrLinePos;
	int iCommand;
	float fParam1;
	bool bReturn = false; ///setting this to true WILL cause auto parsing to quit!
	string rStatus;

	// trim uninteresting stuff from front of string

	rStatement.erase(0, rStatement.find_first_not_of(" \r\n\t"));

	if(rStatement.empty()) {
		//printf("statement is empty");
		return (bReturn);
	}

	// process the autonomous motion

	pCurrLinePos = (char *) rStatement.c_str();

	if(*pCurrLinePos == sComment)
	{
		return (bReturn);
	}

	// find first token

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		printf("%s %s\n", pCurrLinePos, szDelimiters);
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		PRINTAUTOERROR;
		rStatus.append("missing token");
		printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
		return (true);
	}

	// which command are we to execute??
	// this can be (easily) be made much faster... any student want to improve on this?

	for(iCommand = AUTO_TOKEN_MODE; iCommand < AUTO_TOKEN_LAST; iCommand++)
	{
		if(bModeFound)
			printf("Comparing %s to %s\n", pToken, szTokens[iCommand]);

		if(!strncmp(pToken, szTokens[iCommand], strlen(szTokens[iCommand])))
		{
			break;
		}
	}

	if(iCommand == AUTO_TOKEN_LAST)
	{
		// no valid token found
		rStatus.append("no tokens - check script spelling");
		//printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
		return (true);
	}

	// if we are paused wait here before executing a real command

	while(bPauseAutoMode)
	{
		Wait(0.02);
	}

	// execute the proper command

	if(iAutoDebugMode)
	{
		printf("%0.3lf %s %s\n", pDebugTimer->Get(), pToken, pCurrLinePos);
	}

	switch (iCommand)
	{
	case AUTO_TOKEN_MODE:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if(!strncmp(pToken, szModeString, 4))
		{
			// this is our mode
			bModeFound = true;
		}
		else
		{
			// this is not chosen mode
			if (bModeFound)
			{
				// we were executing so time to exit

				End(pCurrLinePos);
				bReturn = true;
			}
		}
		rStatus.append("mode");
		break;

	case AUTO_TOKEN_BEGIN:
		Begin(pCurrLinePos);
		rStatus.append("begin");
		break;

	case AUTO_TOKEN_END:
		End(pCurrLinePos);
		rStatus.append("done");
		bReturn = true;
		break;

	case AUTO_TOKEN_DEBUG:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iAutoDebugMode = atoi(pToken);
		break;

	case AUTO_TOKEN_MESSAGE:
		printf("%0.3lf %03d: %s\n", pDebugTimer->Get(), lineNumber, pCurrLinePos);
		break;

	case AUTO_TOKEN_DELAY:
		if(bModeFound)
		{
			pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

			if (pToken == NULL)
			{
				rStatus.append("missing parameter");
			}
			else
			{
				fParam1 = atof(pToken);
				rStatus.append("wait");

				Delay(fParam1);
			}
		}
		break;

	case AUTO_TOKEN_MOVE:
		if(bModeFound)
		{
			if (!Move(pCurrLinePos))
			{
				rStatus.append("move error");
			}
			else
			{
				rStatus.append("move");
			}
		}
		break;

	case AUTO_TOKEN_MMOVE:
		if(bModeFound)
		{
			if (!MeasuredMove(pCurrLinePos))
			{
				rStatus.append("move error");
			}
			else
			{
				rStatus.append("move");
			}
		}
		break;

	case AUTO_TOKEN_VMOVE:
		if(bModeFound)
		{
			if (!VelocityMove(pCurrLinePos))
			{
				rStatus.append("larger move error");
			}
			else
			{
				rStatus.append("larger move");
			}
		}
		break;

	case AUTO_TOKEN_TURN:
		if(bModeFound)
		{
			if (!Turn(pCurrLinePos))
			{
				rStatus.append("turn error");
			}
			else
			{
				rStatus.append("turn");
			}
		}
		break;

	case AUTO_TOKEN_ELEVATOR:
		if(bModeFound)
		{
			if (!Elevator(pCurrLinePos))
			{
				rStatus.append("elevator error");
			}
			else
			{
				rStatus.append("elevator");
			}
		}
		break;

	case AUTO_TOKEN_CLAW:
		if(bModeFound)
		{
			if (!Claw(pCurrLinePos))
			{
				rStatus.append("claw error");
			}
			else
			{
				rStatus.append("claw");
			}
		}
		break;

	case AUTO_TOKEN_ARM:
		if(bModeFound)
		{
			printf("process AUTO_TOKEN_ARM\n");
			if (!Arm(pCurrLinePos))
			{
				rStatus.append("claw error");
			}
			else
			{
				rStatus.append("claw");
			}
		}
		break;

	case AUTO_TOKEN_PUNCH:
		if(bModeFound)
		{
			printf("process AUTO_TOKEN_PUNCH\n");
			if (!Punch(pCurrLinePos))
			{
				rStatus.append("punch error");
			}
			else
			{
				rStatus.append("punch");
			}
		}
		break;

	default:
		rStatus.append("unknown token");
		break;
	}

	if(bReturn)
	{
		printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
	}

	return (bReturn);
}




