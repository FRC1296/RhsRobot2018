/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

/**
	A template class for creating new components
 */

#include <pthread.h>
#include <string>
#include "RhsRobotBase.h"
#include "RobotMessage.h"

//Robot
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "ComponentBase.h"			//For ComponentBase class

class Drivetrain : public ComponentBase
{
public:
	Drivetrain();
	virtual ~Drivetrain();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Drivetrain *)pThis)->DoWork();
		return(NULL);
	}

private:
	TalonSRX* pLeftMotor;
	TalonSRX* pRightMotor;

	VictorSPX* pLeftSlave1;
	VictorSPX* pLeftSlave2;
	VictorSPX* pRightSlave1;
	VictorSPX* pRightSlave2;

	void OnStateChange();
	void Run();

	int iTicks;
	int iFinalPosLeft;
	int iFinalPosRight;
};

#endif			//COMPONENT_H
