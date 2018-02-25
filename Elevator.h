/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef ELEVATOR_H
#define ELEVATOR_H
#define ACCEPT_RANGE_ELE			768

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

	Timer* pEleTimeout;

	void OnStateChange();
	void Run();

	float fCurVoltage;

	int iCurrPos;
	int iStartPos;
	int iMoveDelta;

	float fMotorSpeed;
	float fMaxSpeed;

	const int iFloorToSwitch = 8000;
	const int iFloorToScale = 21500;
	const int iFloorToClimb = 17500;
	const int iFloorToMax = 25500;
};

#endif			//COMPONENT_H
