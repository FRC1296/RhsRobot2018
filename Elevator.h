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

	void OnStateChange();
	void Run();

	int iLeftInit;
	float fCurVoltage;

	int iTicks;

	float fMotorSpeed;

	// Arbitrary Numbers until we measure
	const int iFloorToSwitch = 100;
	const int iFloorToLowScale = 300;
	const int iFloorToMidScale = 350;
	const int iFloorToHighScale = 400;
	const int iFloorToClimb = 500;
	const int iFloorToMax = 600;
};

#endif			//COMPONENT_H
