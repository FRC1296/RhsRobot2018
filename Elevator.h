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
	TalonSRX* pElevatorMotor;

	void OnStateChange();
	void Run();
};

#endif			//COMPONENT_H
