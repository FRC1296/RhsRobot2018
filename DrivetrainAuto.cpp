/*
 * DrivetrainAuto.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: tkb
 */

#include "Component.h"
#include "ComponentBase.h"
#include "RobotParams.h"
#include "WPILib.h"
#include "Drivetrain.h"
#include "RobotMessage.h"
#include "RhsRobotBase.h"

void Drivetrain::AutoMeasuredMove()
{
	fMMoveTime = localMessage.params.mmove.fTime;
	iTargetDistance = localMessage.params.mmove.fDistance;
	iTicks = (iTargetDistance*4096)/(PI*WHEEL_DIA);
	iFinalPosLeft = pLeftMotor->GetSelectedSensorPosition(0) - iTicks;
	iFinalPosRight = pRightMotor->GetSelectedSensorPosition(0) - iTicks;

	printf("AutoMeasuredMove time %0.3f distance %d ticks %d left %d right %d\n",
			fMMoveTime, iTargetDistance, iTicks, iFinalPosLeft, iFinalPosRight);

	pPIDTimerMove->Reset();
	pPIDTimerMove->Start();

	pLeftMotor->ConfigPeakOutputForward(localMessage.params.mmove.fSpeed, 0.0);
	pLeftMotor->ConfigPeakOutputReverse(-localMessage.params.mmove.fSpeed, 0.0);
	pRightMotor->ConfigPeakOutputForward(localMessage.params.mmove.fSpeed, 0.0);
	pRightMotor->ConfigPeakOutputReverse(-localMessage.params.mmove.fSpeed, 0.0);

	pLeftMotor->Set(ControlMode::Position,iFinalPosLeft);
	pRightMotor->Set(ControlMode::Position,iFinalPosRight);

	SmartDashboard::PutNumber("iFinalPosLeft",iFinalPosLeft);
	SmartDashboard::PutNumber("iFinalPosRight",iFinalPosRight);
	SmartDashboard::PutNumber("iTicks", iTicks);

	while(true)
	{
		// have we timed out?

		if (pPIDTimerMove->Get() >= fMMoveTime)
		{
			break;
		}

		// check to see if we have arrived

		if ((pLeftMotor->GetSelectedSensorPosition(0) <= iFinalPosLeft + ACCEPT_RANGE_MOVE) &&
				(pLeftMotor->GetSelectedSensorPosition(0) >= iFinalPosLeft - ACCEPT_RANGE_MOVE) &&
				(pRightMotor->GetSelectedSensorPosition(0) <= iFinalPosRight + ACCEPT_RANGE_MOVE) &&
				(pRightMotor->GetSelectedSensorPosition(0) >= iFinalPosRight - ACCEPT_RANGE_MOVE))
			{
				break;
			}

		SmartDashboard::PutNumber("Target Left Motor Position",iFinalPosLeft);
		SmartDashboard::PutNumber("Target Right Motor Position",iFinalPosRight);
		Wait(0.02);
	}

	pPIDTimerMove->Stop();

	pLeftMotor->Set(ControlMode::PercentOutput ,0.0);
	pRightMotor->Set(ControlMode::PercentOutput, 0.0);

	pLeftMotor->ConfigPeakOutputForward(1.0, 0.0);
	pLeftMotor->ConfigPeakOutputReverse(-1.0, 0.0);
	pRightMotor->ConfigPeakOutputForward(1.0, 0.0);
	pRightMotor->ConfigPeakOutputReverse(-1.0, 0.0);

	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Drivetrain::AutoMeasuredTurn()
{
	PigeonIMU::GeneralStatus genStatus;

	fTarget = localMessage.params.turn.fAngle;
	iTurnState = TurnState_gpTurn;
	pPIDTurnTimer->Reset();
	pPIDTurnTimer->Start();
	fInitRotation = deg[2];
	fTargetCalc = fInitRotation + fTarget;

printf("AutoMeasuredTurn %d\n", 1);

	SmartDashboard::PutString("PID turn", "TURN PID START");
	SmartDashboard::PutString("Modes","PID Turn Initiated");
	Wait(0.02);

	while(true)
	{
		pIdgey->GetGeneralStatus(genStatus);
		pIdgey->GetAccumGyro(deg);

		// give up if we are taking more than 3 seconds

		if (pPIDTurnTimer->Get() >= 3)
		{
			printf("AutoMesaruedTurn %d\n", 2);

			SmartDashboard::PutString("PID turn","PID Timeout");
			SmartDashboard::PutString("Completed","PID Completed");
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = TurnState_Init;
			fTarget = 0;
			fInitRotation = 0;
			break;
		}

		if ((deg[2] <= (fTargetCalc + ACCEPT_RANGE_DEGR)) && (deg[2] >= (fTargetCalc - ACCEPT_RANGE_DEGR)))
		{
			printf("AutoMeasuredTurn %d\n", 3);

			// we are close enough

			pPIDTimer->Stop();
			SmartDashboard::PutString("Completed","PID Completed");
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = TurnState_Init;
			fTarget = 0;
			fInitRotation = 0;
			break;
		}

		// scale turning rate to amount left to turn

		printf("AutoMeasuredTurn %d\n", 4);

		fP = (fTargetCalc) - deg[2];
		if  (fPrevP == 0)
			fD = 0;
		else
			fD = fPrevP - fP;

		fPrevP = fP;
		fSpeed = (DRIVETRAIN_CONST_KP*fP) - (DRIVETRAIN_CONST_KD*fD);

		if ((deg[2] <= (fTargetCalc + ACCEPT_RANGE_KI)) && (deg[2] >= (fTargetCalc - ACCEPT_RANGE_KI)))
		{
			printf("AutoMeasuredTurn %d\n", 5);

			fI += fP;
			fSpeed += (DRIVETRAIN_CONST_KI*fI);
		}
		else
		{
			fI = 0;
		}

		// limit the max speed

		if (fSpeed < -1 * MAX_SPEED_PID)
		{
			printf("AutoMeasuredTurn %d\n", 6);

			fSpeed = -1 * MAX_SPEED_PID;
		}
		else if (fSpeed > MAX_SPEED_PID)
		{
			printf("AutoMeasuredTurn %d\n", 7);

			fSpeed = MAX_SPEED_PID;
		}

		pLeftMotor->Set(ControlMode::PercentOutput,fSpeed);
		pRightMotor->Set(ControlMode::PercentOutput,-1*fSpeed);

		SmartDashboard::PutString("Modes","PID Running");
		SmartDashboard::PutNumber("P Value",fP);
		SmartDashboard::PutNumber("I Value",fI);
		SmartDashboard::PutNumber("D Value",fD);
		SmartDashboard::PutNumber("Speed",fSpeed);
		SmartDashboard::PutString("Completed","PID Not Completed");

		printf("AutoMeasuredTurn %d\n", 8);

		Wait(0.02);
	}

	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}


