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

enum TurnState{ TurnState_Init = -1, TurnState_mTurn = 1, TurnState_gpTurn, TurnState_boxTurn, TurnState_mMove};
enum ArcState { ArcState_Init = -1, ArcState_Arc = 1, ArcState_Straight, ArcState_Stop };

const double METERS_PER_INCH = 0.0254;
const double METERS_PER_COUNT = (METERS_PER_INCH * 4096)/(PI*WHEEL_DIA);

class CheesyLoop;

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
	Timer* pPunchTimerLeft;
	Timer* pPunchTimerRight;

	Solenoid* pPunchSolenoidLeft; // The solenoid that punches the cube left, NOT the left solenoid
	Solenoid* pPunchSolenoidRight; // The solenoid that punches the cube right, NOT the right solenoid

	Solenoid* pClimberSolenoid;	// The solenoid used to deploy the climber (same as old punching right solenoid)

	Servo* pLeftServo;	// The servo on the left side of the robot
	Servo* pRightServo;	// The servo on the right side of the robot

	void OnStateChange();
	void Run();
//	void BoxCarFilter();
	void GyroPIDTurn();
	void MeasuredTurn();
	void MeasuredMove();
	void AutoMeasuredMove();
	void AutoMeasuredTurn();
	void RunCheesyDrive(bool, float, float, bool);
	void AutoVelocityMove();
	void AutoPunchWhileMovingStraight(bool dir); // Right is true
	void AutoArc(float deg, float radius, float time, bool stop);

	void ArcTest();
	void StopTest();
	void StraightTest();

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
	float fVMoveTime;
	float fSPMoveTime;
	float fTurnTime;
	float fTargetCalc;
	float fMoveAngle;
	float fPunchPoint;

	double dfAccumGyroData[3];
	double dfRawGyroData[3];
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

	int iArcState;

	float fBatteryVoltage;
	bool bUseCheesyDrive;
	bool bInAuto;
	CheesyLoop *pCheesy;
};

#endif			//COMPONENT_H
