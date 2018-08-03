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
#include <Math.h>

void Drivetrain::AutoArc(float deg, float radius, float time, bool stop)
{
	deg = 180.0; // For testing
	radius = 20.0; // For testing
	time = 3.5;
	stop = true;
	float fTicks = (deg*2.0*PI*radius*4096.0)/(360.0*PI*WHEEL_DIA);
	float fAvgVelocity = fTicks / time;
	float fVelOffset = 0.0;
	float fFinalLeft = pLeftMotor->GetSelectedSensorPosition(0) - fTicks;
	float fFinalRight = pRightMotor->GetSelectedSensorPosition(0) - fTicks;
	float fCurAngle;
	float fTargetAngle;
	float fPGL;
	float fPGR;
	float fSpeedLeft;
	float fSpeedRight;
	const float kPPL = .66;
	const float kPPR = .66;
	const float kPGL = 650;
	const float kPGR = 650;
	float kDGL = .1;
	float kDGR = .1;
	float fPPL;
	float fPPR;

	pPIDTimerMove->Reset();
	pPIDTimerMove->Start();

	while (true)
	{
		pIdgey->GetAccumGyro(dfAccumGyroData);
		fCurAngle = dfAccumGyroData[2];

		if (!bInAuto)
		{
			break;
		}

		if (pPIDTimerMove->Get() > time)
		{
			break;
		}

		// check to see if we have arrived
		if ((pLeftMotor->GetSelectedSensorPosition(0) <= iFinalPosLeft + ACCEPT_RANGE_MOVE) &&
				(pLeftMotor->GetSelectedSensorPosition(0) >= iFinalPosLeft - ACCEPT_RANGE_MOVE) &&
				(pRightMotor->GetSelectedSensorPosition(0) <= iFinalPosRight + ACCEPT_RANGE_MOVE) &&
				(pRightMotor->GetSelectedSensorPosition(0) >= iFinalPosRight - ACCEPT_RANGE_MOVE))
		{
			fPPL = fPPR = 0;
			break;
		}

		// This all needs to be changed below here but I haven't slept so I'll figure it out later

		// Setting the target angle to the intended angle
		fTargetAngle = fCurAngle + deg;

		// Getting the positional relation
		fPPL = fFinalLeft - pLeftMotor->GetSelectedSensorPosition(0);
		fPPR = fFinalRight - pRightMotor->GetSelectedSensorPosition(0);

		// Directly from AutoVelocityMove... will probably change
		fSpeedLeft = (fPPL * kPPL) + (fPGL * kPGL) - (fPGL * kDGL);
		fSpeedRight = (fPPR * kPPR) + (fPGR * kPGR) - (fPGR * kDGR);

		// Directly from AutoVelocityMove... will probably changes
		pLeftMotor->Set(ControlMode::Velocity,fSpeedLeft);
		pRightMotor->Set(ControlMode::Velocity,fSpeedRight);

		Wait(0.02);
	}

	pPIDTimerMove->Stop();

	if (stop)
	{
		pLeftMotor->Set(ControlMode::PercentOutput ,0.0);
		pRightMotor->Set(ControlMode::PercentOutput, 0.0);
	}

	pLeftMotor->ConfigPeakOutputForward(1.0, 0.0);
	pLeftMotor->ConfigPeakOutputReverse(-1.0, 0.0);
	pRightMotor->ConfigPeakOutputForward(1.0, 0.0);
	pRightMotor->ConfigPeakOutputReverse(-1.0, 0.0);

	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Drivetrain::ArcTest() {
	/* Alright so here's my thought process:
	 * Assuming one continuous motion (obviously impossible because acceleration / friction), then
	 * The assumed path of travel is off by +- a radius of 13 inches to each side of the robot.
	 * Thus, take the assumed path of travel's velocity and calculate two separate velocities based on
	 * concentric circles exactly thirteen inches smaller and larger in order to get one "arc" motion.
	 */

	float fTime = localMessage.params.turn.driveTime; // assumed time
	float fRadius = localMessage.params.turn.ArcRadius; // radius in inches
	float fWOff = 13.0; // Wheel offset in inches
	float fAngle = localMessage.params.turn.ArcAngle; // Desired angle in degrees, assuming right is positive for now

	int iCenterArcLength = (int)(2*fRadius*fAngle*4096)/(360.0*WHEEL_DIA);

	int iLeftArcLength = (int)(2*(fRadius+fWOff)*fAngle*4096)/(360.0*WHEEL_DIA);
	int iRightArcLength = (int)(2*(fRadius-fWOff)*fAngle*4096)/(360.0*WHEEL_DIA);

	float fLeftVelocity = (iLeftArcLength / fTime)*-1;
	float fRightVelocity = (iRightArcLength / fTime)*-1;

	if (pPIDTimerMove->Get() >= fTime) {
		iArcState = ArcState_Init;
		pPIDTimerMove->Stop();
		pPIDTimerMove->Reset();
		pLeftMotor->Set(ControlMode::PercentOutput,0.0);
		pRightMotor->Set(ControlMode::PercentOutput,0.0);
		return;
	}

	pLeftMotor->Set(ControlMode::Velocity,fLeftVelocity);
	pRightMotor->Set(ControlMode::Velocity,fRightVelocity);

}

void Drivetrain::StopTest() {
	static const float fTime = 0.5;
	if (pPIDTimerMove->Get() >= fTime) {
		iArcState = ArcState_Init;
		pPIDTimerMove->Stop();
		pPIDTimerMove->Reset();
		pLeftMotor->Set(ControlMode::PercentOutput,0.0);
		pRightMotor->Set(ControlMode::PercentOutput,0.0);
		return;
	}
	pLeftMotor->Set(ControlMode::PercentOutput,0.0);
	pRightMotor->Set(ControlMode::PercentOutput,0.0);
}

void Drivetrain::StraightTest() {
	SmartDashboard::PutString("New Auto Test","Going Straight");
	float fTime = localMessage.params.mmove.fTime;
	float fDistance = localMessage.params.mmove.fDistance;
	float fTarget = (fDistance * 4096)/(PI*WHEEL_DIA);

	float fVelocity = (fTarget / fTime)*-1;

	if (pPIDTimerMove->Get() >= fTime) {
		iArcState = ArcState_Init;
		pPIDTimerMove->Stop();
		pPIDTimerMove->Reset();
		pLeftMotor->Set(ControlMode::PercentOutput,0.0);
		pRightMotor->Set(ControlMode::PercentOutput,0.0);
		SmartDashboard::PutString("New Auto Test","Completed");
		return;
	}

	pLeftMotor->Set(ControlMode::Velocity,fVelocity);
	pRightMotor->Set(ControlMode::Velocity,fVelocity);

}
