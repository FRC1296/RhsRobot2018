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
#include <math.h>
#include "RobotMessage.h"
#include "RobotParams.h"

//Robot
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "ComponentBase.h"			//For ComponentBase class

void AddToArray(int* Array, int LengthArr, int val);
float AvgArrays(int* Array, int LengthArr);
void dAddToArray(float* Array, int LengthArr, float val);
float dAvgArrays(float* Array, int LengthArr);

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
	TalonSRX* pLeftSlave2;
	VictorSPX* pRightSlave1;
	VictorSPX* pRightSlave2;

	PigeonIMU* pIdgey;

	Timer* pPIDTimer;
	Timer* pPIDTimerMove;
	Timer* pSpeedTimer;
	Timer* pPIDTurnTimer;

	void OnStateChange();
	void Run();
	void BoxCarFilter();
	void GyroPIDTurn();
	void MeasuredTurn();
	void MeasuredMove();

	float fInitRotation;
	float fPrevP;
	float fSpeed;
	float fCurrentPos;
	float fTarget;
	float fP;
	float fD;
	float fI;
	float fMaxTurnSpeed;
	float fMaxStraightSpeed;
	float fTurnTTM;
	float fStraightTTM;
	float fMaxTurnX;
	float fMaxTurnY;
	float fMaxTurnZ;
	float fTimeToDest;
	float dAvgArray1;
	float dAvgArray2;
	float fMMoveTime;
	float fTargetCalc;

	double deg[3];
	double dps[3];

	int iTurnState;
	int iTicks;
	int iFinalPosLeft;
	int iFinalPosRight;
	int iTurnArray[FILTER_ONE_LENGTH];
	float dTurnArray2[FILTER_TWO_LENGTH];
	int iNumPoints;
	int iCurrNumPoints;
	int iTargetDistance;

	int iInitLeftPos;
	int iInitRightPos;
};

#endif			//COMPONENT_H
