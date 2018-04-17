/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef CLIMBER_H
#define CLIMBER_H

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


class Climber : public ComponentBase
{
public:
	Climber();
	virtual ~Climber();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Climber *)pThis)->DoWork();
		return(NULL);
	}

private:
	float fSpeed;

	TalonSRX* pUpMotor;
	TalonSRX* pUpSlave;
	void OnStateChange();
	void Run();
};

#endif			//COMPONENT_H
