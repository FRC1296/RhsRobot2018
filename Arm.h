
#ifndef ARM_H
#define ARM_H

/**
	A template class for creating new components
 */
#include "ComponentBase.h"
#include <pthread.h>
#include <string>
#include "RhsRobotBase.h"
#include "RobotMessage.h"

//Robot
#include "WPILib.h"
#include "ctre\Phoenix.h"


class Arm : public ComponentBase
{
public:
	Arm();
	virtual ~Arm();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Arm *)pThis)->DoWork();
		return(NULL);
	}

private:
	TalonSRX* pArmMotor;

	Timer* pArmTimeout;

	void OnStateChange();
	void Run();

	float fCurVoltage;

	int iCurrPos;
	int iStartPos;
	int iMoveDelta;

	float fMotorSpeed;
	float fMaxSpeed;

	const int iStartToOpen = 3500;
	const int iStartToStow = 1500;
	const int iStartToShoot = 1500;
	const int iStartToMax = 3700;
};

#endif			//ARM_H
