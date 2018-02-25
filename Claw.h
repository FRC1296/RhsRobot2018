/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef CLAW_H
#define CLAW_H

/**
	A template class for creating new components
 */
#include "ComponentBase.h"			//For ComponentBase class
#include <pthread.h>
#include <string>
#include "RhsRobotBase.h"
#include "RobotMessage.h"
#include "RobotParams.h"

//Robot
#include "WPILib.h"
#include "ctre\Phoenix.h"
#include "Solenoid.h"

const int CLAW_LIMIT = 20; // Claw's current limit


class Claw : public ComponentBase
{
public:
	Claw();
	virtual ~Claw();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Claw *)pThis)->DoWork();
		return(NULL);
	}

private:
	VictorSPX* pClawVictorLeft;
	VictorSPX* pClawVictorRight;
	Solenoid* pClawSolenoidLeft;
	Solenoid* pClawSolenoidRight;

	PowerDistributionPanel* pPDP;

	bool motorsStopped;

	void OnStateChange();
	void Run();
};

#endif			//COMPONENT_H

