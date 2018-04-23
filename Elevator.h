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

	bool LimitSpeed();
	float PercentHeight();
	bool AtFloorPos();

private:
	TalonSRX* pElevatorMotorLeft;
	TalonSRX* pElevatorMotorRight;

	DigitalInput* pSlowHallEffect; // aka Kelvin
	DigitalInput* pStopHallEffect; // aka Alex

	Timer* pEleTimeout;

	void OnStateChange();
	void Run();
	int Zero();

	float fCurVoltage;

	int iCurrPos;
	int iCurrTgt;
	int iStartPos;
	int iMoveDelta;
	int iPrevDelta;
	int iHoldPos;
	int iCurrentPid;

	float fMotorSpeed;
	float fMaxSpeed;

	bool bFloorPos;
	bool prevPressed;
	bool prevActivated;

	const int iFloorToSwitch = 11798;
	const int iFloorToSpeedLimit = 17788; // Needs measuring; right now it's a rough estimate
	const int iFloorToScale = 22703;
	const int iFloorToClimb = 17500; // Needs updating
	const int iFloorToMax = 25250;
	const int iSwitchToScale = (iFloorToScale - iFloorToSwitch);
	const int iMoveDeltaIncrement = 100;
};

#endif			//COMPONENT_H
