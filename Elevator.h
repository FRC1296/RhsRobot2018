/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef ELEVATOR_H
#define ELEVATOR_H

/**
	A template class for creating new components
 */
#include "ComponentBase.h"			//For ComponentBase class
#include <pthread.h>
#include <string>
#include "RhsRobotBase.h"
#include "RobotMessage.h"

//Robot
#include "WPILib.h"
#include "ctre\Phoenix.h"


class Elevator : public ComponentBase
{
enum EleState{ EleState_Init = -1, EleState_Floor = 1, EleState_Switch, EleState_Scale, EleState_Climb};
public:
	Elevator();
	virtual ~Elevator();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Elevator *)pThis)->DoWork();
		return(NULL);
	}

private:
	TalonSRX* pElevatorMotorLeft;
	TalonSRX* pElevatorMotorRight;

	Timer* pEleTimer;
	Timer* pEleTimeout;

	void OnStateChange();
	void Run();
	void Floor(int iCurrPos);
	void Switch(int iCurrPos);
	void Scale(int iCurrPos);

	int iLeftInit;
	float fCurVoltage;
	bool bEnable;
	bool bFloor;

	int iCurrPos;
	int iStartPos;
	int iStopPos;
	int iEleState;

	float fMotorSpeed;
	float fMaxSpeed;

	const int iFloorToSwitch = 16000/*-4362*/;
	const int iFloorToScale = 35000;
	const int iFloorToClimb = 500;
	const int iFloorToMax = 600;
};

#endif			//COMPONENT_H
