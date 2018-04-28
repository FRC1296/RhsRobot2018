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

void Drivetrain::AutoArc(float deg, float radius, float time, bool stop) {
	deg = 180.0; // For testing
	radius = 20.0; // For testing
	time = 3.5;
	stop = true;
	float fTicks = (deg*2.0*PI*radius*4096.0)/(360.0*PI*WHEEL_DIA);
	float fAvgVelocity = fTicks / time;
	float fVelOffset = 0.0;
	float fFinalLeft = pLeftMotor->GetSelectedSensorPosition(0);
	float fFinalRight = pRightMotor->GetSelectedSensorPosition(0);
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
	float kPDL = .1;
	float kPDR = .1;
	float fPPL;
	float fPPR;

	if (!stop) {
		kPDL = kPDR = 0; // all gas no brakes
	}

	pIdgey->GetAccumGyro(dfAccumGyroData);
	fCurAngle = dfAccumGyroData[2];
	fTargetAngle = fCurAngle + deg;
}
