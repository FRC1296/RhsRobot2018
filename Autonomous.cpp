/** \filef
 * Class for our autonomous behaviours
 *
 *  This file contains our autonomous algorithms.  It should detect if we are in
 *  autonomous mode or not, select an algorithm based upon switch settings at
 *  the driver station and implement the behaviours till autonomous mode ends.
 */

#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include "WPILib.h"

//Robot
#include "RobotParams.h"
#include "Autonomous.h"
#include "AutoParser.h"
#include "ComponentBase.h"


using namespace std;

extern "C" {
}

bool Autonomous::CommandResponse(const char *szQueueName) {
	int iPipeXmt;
	bool bReturn = true;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	bReceivedCommandResponse = false;
	Message.replyQ = AUTONOMOUS_QUEUE;
	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);

	while (!bReceivedCommandResponse)
	{
		//purposefully empty
	}

	if(iAutoDebugMode)
	{
		printf("%0.3lf Response received\n", pDebugTimer->Get());
	}

	if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
	{
		//		SmartDashboard::PutString("Auto Status","auto ok");
		bReturn = true;
	}
	else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		PRINTAUTOERROR;
		bReturn = false;
	}

	return bReturn;
}


//UNTESTED
//USAGE: MultiCommandResponse({DRIVETRAIN_QUEUE, CONVEYOR_QUEUE}, {COMMAND_DRIVETRAIN_STRAIGHT, COMMAND_CONVEYOR_SEEK_TOTE});
bool Autonomous::MultiCommandResponse(vector<char*> szQueueNames, vector<MessageCommand> commands) {
	//wait for several commands at once
	//check that queue list is as long as command list
	if(szQueueNames.size() != commands.size())
	{
		//		SmartDashboard::PutString("Auto Status","MULTICOMMAND error!");
		return false;
	}
	bool bReturn = true;
	int iPipeXmt;
	uResponseCount = 0;
	//vector<int> iPipesXmt = new vector<int>();

	bReceivedCommandResponse = false;

	//send messages to each component
	for (unsigned int i = 0; i < szQueueNames.size(); i++)
	{
		iPipeXmt = open(szQueueNames[i], O_WRONLY);
		wpi_assert(iPipeXmt > 0);

		Message.replyQ = AUTONOMOUS_QUEUE;
		Message.command = commands[i];
		write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
		close(iPipeXmt);
	}

	while (uResponseCount < szQueueNames.size())
	{
		while (!bReceivedCommandResponse)
		{
			//purposefully empty
		}

		if(iAutoDebugMode)
		{
			printf("%0.3lf Response received\n", pDebugTimer->Get());
		}

		if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
		{
			//			SmartDashboard::PutString("Auto Status", "auto ok");
			bReturn = true;
		}
		else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
		{
			//			SmartDashboard::PutString("Auto Status", "EARLY DEATH!");
			bReturn = false;
		}
	}
	return bReturn;
}

bool Autonomous::CommandNoResponse(const char *szQueueName) {
	int iPipeXmt;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);
	return (true);
}

void Autonomous::Delay(float delayTime)
{
	double fWait;


	for (fWait = 0.0; fWait < delayTime; fWait += 0.01)
	{
		// if we are paused break the delay into pieces
		while (bPauseAutoMode)
		{
			Wait(0.02);
		}

		Wait(0.01);
	}
}

bool Autonomous::Begin(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_RUN;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::End(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is ending
	Message.command = COMMAND_AUTONOMOUS_COMPLETE;
	CommandNoResponse(DRIVETRAIN_QUEUE);
	return (true);
}

bool Autonomous::Move(char *pCurrLinePos) {
	char *pToken;
	float fLeft;
	float fRight;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fLeft = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fRight = atof(pToken);

	Message.command = COMMAND_DRIVETRAIN_RUN_TANK;
	Message.params.tdrive.left =  -fLeft;
	Message.params.tdrive.right = fRight;

	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::MeasuredMove(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fTime = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_AUTOMOVE;
	Message.params.mmove.fSpeed = fSpeed;
	Message.params.mmove.fDistance = fDistance;
	Message.params.mmove.fTime = fTime;

#ifndef TEST_SCRIPTS
	return (CommandResponse(DRIVETRAIN_QUEUE));
#else
	printf("COMMAND_DRIVETRAIN_MMOVE %0.2f %0.2f %0.2f\n", fSpeed, fDistance, fTime);
	return(true);
#endif
}

bool Autonomous::VelocityMove(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fTime = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_AUTO_LONGMOVE;
	Message.params.mmove.fSpeed = fSpeed;
	Message.params.mmove.fDistance = fDistance;
	Message.params.mmove.fTime = fTime;

#ifndef TEST_SCRIPTS
	return (CommandResponse(DRIVETRAIN_QUEUE));
#else
	printf("COMMAND_DRIVETRAIN_VMOVE %0.2f %0.2f %0.2f\n", fSpeed, fDistance, fTime);
	return(true);
#endif
}

bool Autonomous::Turn(char *pCurrLinePos) {
	char *pToken;
	float fAngle;
	float fTimeout;

	// parse remainder of line to get target angle and timeout
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fAngle = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fTimeout = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_AUTOTURN;
	Message.params.turn.fAngle= fAngle;
	Message.params.turn.fTimeout = fTimeout;

#ifndef TEST_SCRIPTS
	printf("Turn Message to Drivetrain\n");
	return (CommandResponse(DRIVETRAIN_QUEUE));
#else
	printf("COMMAND_DRIVETRAIN_MTURN %0.2f %0.2f\n", fAngle, fTimeout);
	return(true);
#endif
}

bool Autonomous::Arc(char *pCurrLinePos)
{
	char* pToken;
	float fAngle;
	float fRadius;
	float fTime;

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	if (pToken == NULL)
	{
		return (false);
	}

	fAngle = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	if (pToken == NULL)
	{
		return (false);
	}

	fRadius = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	if (pToken == NULL)
	{
		return (false);
	}

	fTime = atof(pToken);

	Message.command = COMMAND_DRIVETRAIN_ARC;
	Message.params.turn.ArcAngle = fAngle;
	Message.params.turn.ArcRadius = fRadius;
	Message.params.turn.driveTime = fTime;
#ifndef TEST_SCRIPTS
	printf("Arc Message to Drivetrain\n");
	return (CommandResponse(DRIVETRAIN_QUEUE));
#else
	printf("COMMAND_DRIVETRAIN_MTURN %0.2f %0.2f\n", fAngle, fTimeout);
	return(true);
#endif
}

bool Autonomous::Straight(char *pCurrLinePos)
{
	char* pToken;
	float fDistance;
	float fTime;

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	if (pToken == NULL)
	{
		return (false);
	}

	fDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	if (pToken == NULL)
	{
		return (false);
	}

	fTime = atof(pToken);

	Message.command = COMMAND_DRIVETRAIN_STRAIGHT;
	Message.params.mmove.fDistance = fDistance;
	Message.params.mmove.fTime = fTime;
#ifndef TEST_SCRIPTS
	printf("Arc Message to Drivetrain\n");
	return (CommandResponse(DRIVETRAIN_QUEUE));
#else
	printf("COMMAND_DRIVETRAIN_MTURN %0.2f %0.2f\n", fAngle, fTimeout);
	return(true);
#endif
}

bool Autonomous::Stop(char *pCurrLinePos)
{
	Message.command = COMMAND_DRIVETRAIN_STOP;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Elevator(char *pCurrLinePos)
{
	char *pToken;
	float fTimeout;

	// parse remainder of line to get mode
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	if(!strncmp(pToken, "INTAKE", 6))
	{
		Message.command = COMMAND_ELEVATOR_FLOOR;
	}
	else if(!strncmp(pToken, "SWITCH", 6))
	{
		Message.command = COMMAND_ELEVATOR_SWITCH;
	}
	else if(!strncmp(pToken, "SCALE", 5))
	{
		Message.command = COMMAND_ELEVATOR_SCALE;
	}
	else if(!strncmp(pToken, "STOW", 4))
	{
		Message.command = COMMAND_ELEVATOR_NOBUTTON;
	}
	else if(!strncmp(pToken, "FLOOR", 5))
	{
		Message.command = COMMAND_ELEVATOR_NOBUTTON;
	}
	else if(!strncmp(pToken, "MAX", 3))
	{
		Message.command = COMMAND_ELEVATOR_CLIMB;
	}
	else
	{
		return(false);
	}

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fTimeout = atof(pToken);
	Message.params.elevator.fTime = fTimeout;

#ifndef TEST_SCRIPTS
	return (CommandNoResponse(ELEVATOR_QUEUE));
#else
	printf("Elevator %s\n", pToken);
	return(true);
#endif
}

bool Autonomous::Claw(char *pCurrLinePos)
{
	char *pToken;

	// parse remainder of line to get mode
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	if(!strncmp(pToken, "IN", 2))
	{
		// parse remainder of line to get the speed
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if(pToken == NULL)
		{
			//			SmartDashboard::PutString("Auto Status","EARLY DEATH!");
			return (false);
		}

		Message.command = COMMAND_CLAW_INHALE;
		Message.params.claw.fClawSpeed = atof(pToken);
	}
	else if(!strncmp(pToken, "OUT", 3))
	{
		// parse remainder of line to get the speed
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if(pToken == NULL)
		{
			//			SmartDashboard::PutString("Auto Status","EARLY DEATH!");
			return (false);
		}

		Message.command = COMMAND_CLAW_EXHALE;
		Message.params.claw.fClawSpeed = atof(pToken);
	}
	else if(!strncmp(pToken, "STOP", 4))
	{
		Message.command = COMMAND_CLAW_STOP;
		Message.params.claw.fClawSpeed = 0.0;
	}
	else
	{
		return(false);
	}

#ifndef TEST_SCRIPTS
	return (CommandNoResponse(CLAW_QUEUE));
#else
	printf("Claw %0.2f\n", Message.params.claw.fClawSpeed);
	return(true);
#endif
}

bool Autonomous::Arm(char *pCurrLinePos)
{
	char *pToken;

	// parse remainder of line to get mode
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		//		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	printf("comparing %s ...\n", pToken);

	if(!strncmp(pToken, "PINCH", 5))
	{
		printf("COMMAND_CLAW_PINCH\n");
		Message.command = COMMAND_CLAW_PINCH;
	}
	else if(!strncmp(pToken, "RELEASE", 7))
	{
		printf("COMMAND_CLAW_RELEASE\n");
		Message.command = COMMAND_CLAW_RELEASE;
	}
	else if(!strncmp(pToken, "FLOOR", 5))
	{
		printf("COMMAND_ARM_FLOOR\n");
		//		SmartDashboard::PutString("Arm Status","Auto GoToFloor");
		Message.command = COMMAND_ARM_FLOOR;
	}
	else if(!strncmp(pToken, "SHOOT", 5))
	{
		printf("COMMAND_ARM_SHOOT\n");
		Message.command = COMMAND_ARM_SHOOT;
	}
	else
	{
		return(false);
	}

#ifndef TEST_SCRIPTS
	return (CommandNoResponse(ARM_QUEUE));
#else
	printf("Claw %0.2f\n", Message.params.claw.fClawSpeed);
	return(true);
#endif
}

bool Autonomous::SPunch(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;
	float fTime;
	float fPunchDistance;

	//	SmartDashboard::PutString("Spunch Status","Autonomous.cpp command called");

	// parse remainder of line to get length to move
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	printf("comparing %s ...\n", pToken);

	if(!strncmp(pToken, "LEFT", 4))
	{
		Message.command = COMMAND_SPUNCH_LEFT;
	}
	else if(!strncmp(pToken, "RIGHT", 5))
	{
		Message.command = COMMAND_SPUNCH_RIGHT;
	}
	else
	{
		return(false);
	}

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH");
		return (false);
	}

	fPunchDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fTime = atof(pToken);

	Message.params.mmove.fPunchDistance = fPunchDistance;
	Message.params.mmove.fSpeed = fSpeed;
	Message.params.mmove.fDistance = fDistance;
	Message.params.mmove.fTime = fTime;

#ifndef TEST_SCRIPTS
	return (CommandResponse(DRIVETRAIN_QUEUE));
#else
	printf("COMMAND_DRIVETRAIN_SPUNCH %0.2f %0.2f %0.2f %0.2f\n", fSpeed, fDistance, fTime);
	return(true);
#endif
}
