/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

/**
	A template class for creating new components
 */
#include "ComponentBase.h"			//For ComponentBase class
#include <pthread.h>
#include <string>

//Robot
#include "WPILib.h"


class Autonomous : public ComponentBase
{
public:
	Autonomous();
	virtual ~Autonomous();
	static void *StartTask(void *pThis, const char* szAutonomousName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szAutonomousName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Autonomous *)pThis)->DoWork();
		return(NULL);
	}

private:
	void OnStateChange();
	void Run();
};

#endif			//COMPONENT_H
