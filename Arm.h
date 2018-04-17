
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
	DigitalInput* pBumperSwitch;

	TalonSRX* pArmMotor;
	Timer* pArmTimeout;

	Solenoid* pClawSolenoid;

	void OnStateChange();
	void Run();
	int ZeroArm();

	float fCurVoltage;

	int iCurrPos;
	int iStartPos;
	int iMoveDelta;
	int iStowPos;
	int iFloorPos;

	float fMotorSpeed;
	float fMaxSpeed;

	bool bClawOpen;
	bool bTogglePressed;

	const int iStartToOpen = 6015;
	const int iStartToStow = 2282;
	const int iStartToShoot = 2282;
	const int iStartToMax = 6115;
};

#endif			//ARM_H
