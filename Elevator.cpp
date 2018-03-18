
#include "Component.h"
#include "ComponentBase.h"
#include "RobotParams.h"
#include "WPILib.h"
#include "Elevator.h"

//Robot


/* servo notes
 *
 * when the elevator goes up the encoder ticks DECREASE for sure
 *
 * joysticks give negative numbers when you press them up, remember they were originally
 * designed as flight controls
 *
 * positive motor motion makes the elevator go up for sure
 *
 * so we need to SetSensorPhase(true) to reverse the direction sense of the encoder ticks
 *
 * we do NOT need to call SetInverted(true)
 *
 * so why was the servo going nuts?  a bug!!  one must call ConfigPeakCurrentDuration
 * if you are using ConfigPeakCurrentLimit!
 *
 */


Elevator::Elevator()
: ComponentBase(ELEVATOR_TASKNAME, ELEVATOR_QUEUE, ELEVATOR_PRIORITY)
{
	//TODO: add member objects
	pElevatorMotorLeft = new TalonSRX(CAN_ELEVATOR_TALON_LEFT);
	pElevatorMotorRight = new TalonSRX(CAN_ELEVATOR_TALON_RIGHT);
	pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
	pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
	pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
	pElevatorMotorRight->Set(ControlMode::PercentOutput,0);
	pElevatorMotorLeft->ConfigPeakCurrentLimit(30,1000);
	pElevatorMotorRight->ConfigPeakCurrentLimit(30,1000);
	pElevatorMotorLeft->ConfigContinuousCurrentLimit(30,1000);
	pElevatorMotorRight->ConfigContinuousCurrentLimit(30,1000);
	pElevatorMotorLeft->ConfigPeakCurrentDuration(0.0,10);
	pElevatorMotorRight->ConfigPeakCurrentDuration(0.0,10);
	pElevatorMotorLeft->EnableCurrentLimit(true);
	pElevatorMotorRight->EnableCurrentLimit(true);
	pElevatorMotorLeft->SetInverted(false);
	pElevatorMotorRight->SetInverted(false);
	pElevatorMotorLeft->ConfigPeakOutputForward(0.82, 10);
	pElevatorMotorLeft->ConfigPeakOutputReverse(-.2,10);
	pElevatorMotorRight->ConfigPeakOutputForward(0.82, 10);
	pElevatorMotorRight->ConfigPeakOutputReverse(-.2,10);

	pElevatorMotorLeft->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,10);
	pElevatorMotorLeft->SetSensorPhase(true);

	pElevatorMotorLeft->Config_kF(0,0.0,10);
	pElevatorMotorLeft->Config_kP(0,0.2,10);
	pElevatorMotorLeft->Config_kI(0,0.0,10);
	pElevatorMotorLeft->Config_kD(0,0.0,10);
	pElevatorMotorLeft->ConfigAllowableClosedloopError(0, 200, 10);

	//pElevatorMotorLeft->Config_kF(1,0.0,10);
	//pElevatorMotorLeft->Config_kP(1,0.2,10);
	//pElevatorMotorLeft->Config_kI(1,0.0,10);
	//pElevatorMotorLeft->Config_kD(1,0.0,10);
	//pElevatorMotorLeft->ConfigAllowableClosedloopError(0, 200, 10);
	pElevatorMotorLeft->Config_kF(1,0.0,10);
	pElevatorMotorLeft->Config_kP(1,0.0,10);
	pElevatorMotorLeft->Config_kI(1,0.2,10);
	pElevatorMotorLeft->ConfigMaxIntegralAccumulator(1, 8092, 10);
	pElevatorMotorLeft->Config_kD(1,0.05,10);
	pElevatorMotorLeft->ConfigAllowableClosedloopError(1, 100, 10);

	iCurrentPid = 0;
	pElevatorMotorLeft->SelectProfileSlot(iCurrentPid,0);

	Wait(1.0);
	pElevatorMotorRight->Set(ControlMode::Follower,CAN_ELEVATOR_TALON_LEFT);
	pElevatorMotorLeft->Set(ControlMode::Position, pElevatorMotorLeft->GetSelectedSensorPosition(0));
	//pElevatorMotorLeft->Set(ControlMode::PercentOutput, 0.0);

	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	iStartPos = iCurrPos;
	iCurrTgt = iCurrPos;
	fCurVoltage = 0;
	fMotorSpeed = 0;
	fMaxSpeed = 0;
	iMoveDelta = 0;  // for now
	iPrevDelta = 0;
	iHoldPos = 0;

	pEleTimeout = new Timer();

	SmartDashboard::PutNumber("Elevator Speed RPM",((fMaxSpeed*600)/4096.0));
	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);

	SmartDashboard::PutNumber("Elevator Init Position",iStartPos);
	SmartDashboard::PutNumber("Elevator Switch Position",iStartPos + iFloorToSwitch);
	SmartDashboard::PutNumber("Elevator Scale Position",iStartPos + iFloorToScale);

	pTask = new std::thread(&Elevator::StartTask, this, ELEVATOR_TASKNAME, ELEVATOR_PRIORITY);
	wpi_assert(pTask);
};

Elevator::~Elevator()
{
	//TODO delete member objects

	delete(pTask);
};

void Elevator::OnStateChange()
{
};

void Elevator::Run()
{
	static bool prevPressed = false;
	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);

	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(iCurrentPid);
	iCurrTgt = pElevatorMotorLeft->GetClosedLoopTarget(iCurrentPid);

	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);
	SmartDashboard::PutNumber("MoveDelta",iMoveDelta);

	if((iCurrTgt - iCurrPos) > 200)
	{
		// change the PIS cojnstants for upward motion
	SmartDashboard::PutNumber("MoveDelta",iMoveDelta);

		if(iCurrentPid == 1)
		{
			printf("going up\n");
		}

		iCurrentPid = 0;
	}
	else if((iCurrTgt - iCurrPos) < -200)
	{
		// change the PID constants for downward motion

		if(iCurrentPid == 0)
		{
			printf("going up\n");
		}

		iCurrentPid = 1;
	}

	pElevatorMotorLeft->SelectProfileSlot(iCurrentPid,0);


	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_ELEVATOR_MOVE:
		fMotorSpeed = localMessage.params.elevator.fSpeed;

		if (fMotorSpeed > 0.2 && iCurrPos > iStartPos)
		{
			iMoveDelta -= iMoveDeltaIncrement;
		}
		else if(fMotorSpeed < -0.2 && iCurrPos < iStartPos + iFloorToMax)
		{
			iMoveDelta += iMoveDeltaIncrement;
		}
		break;

	case COMMAND_ELEVATOR_NOBUTTON:
		prevPressed = false;
		iMoveDelta = 0;
		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Coast);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Coast);
		pElevatorMotorLeft->Set(ControlMode::Position, iStartPos + iMoveDelta);
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_FLOOR:
		if(!prevPressed) iMoveDelta = 0;
		prevPressed = true;

		if(iPrevDelta== iMoveDelta)
		{
			iMoveDelta = 0;
			if(iHoldPos == 0)
				iHoldPos = iCurrPos;
			//break;
		}
		else
		{
			iHoldPos = 0;
		}

		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Coast);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Coast);

		if(( (iCurrPos + iMoveDelta) < (iStartPos + iFloorToMax)) &&( (iCurrPos + iMoveDelta) > iStartPos))
		{
			if(iMoveDelta==0)
			{
				pElevatorMotorLeft->Set(ControlMode::Position, iHoldPos + iMoveDelta);
			}
			else
			{
				pElevatorMotorLeft->Set(ControlMode::Position, iCurrPos + iMoveDelta);
			}
		}
		else
		{
			pElevatorMotorLeft->Set(ControlMode::Position,iCurrPos);
		}

		pEleTimeout->Reset();
		pEleTimeout->Start();

		iPrevDelta = iMoveDelta;
		break;

	case COMMAND_ELEVATOR_SWITCH:
		prevPressed = false;

		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);

		if( (iStartPos + iFloorToSwitch + iMoveDelta) > iFloorToMax)
		{
			pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorLeft->Set(ControlMode::Position, iFloorToMax);
		}
		else
		{
			pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToSwitch + iMoveDelta);
		}

		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_SCALE:
		prevPressed = false;

		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);

		if( (iStartPos + iFloorToScale + iMoveDelta) > iFloorToMax)
		{
			pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorLeft->Set(ControlMode::Position, iFloorToMax);
		}
		else
		{
			pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToScale + iMoveDelta);
		}

		SmartDashboard::PutString("Button", "Y Button Pushed");
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_CLIMB:
		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);

		prevPressed = false;
		//	pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToClimb);
		break;

	default:
		prevPressed = false;
		break;
	}

	// implement timeout

	SmartDashboard::PutNumber("Closed Loop Error", pElevatorMotorLeft->GetClosedLoopError(0));

	if((pElevatorMotorLeft->GetClosedLoopError(0) < 200) && (pElevatorMotorLeft->GetClosedLoopError(0) > -200))
	{
		// the servo has closed
		pEleTimeout->Reset();
		pEleTimeout->Stop();
	}
	else
	{
		if(pEleTimeout->Get() > 3)
		{
			// use the current position which should stop the servo
			pElevatorMotorLeft->Set(ControlMode::Position, iCurrPos);
		}
	}

};

bool Elevator::LimitSpeed() // Very quick and dirty speed limit function
{
	if (pElevatorMotorLeft->GetSelectedSensorPosition(0) - iStartPos > iFloorToSpeedLimit)
	{
		return true;
	}
	return false;
}

float Elevator::PercentHeight() // Very quick and dirty function to get elevator's height as % of scale height
{
	float fPercent = 0.0;
	float fCurPos = pElevatorMotorLeft->GetSelectedSensorPosition(0) - iStartPos;
	float fLimitLow = (float)(pElevatorMotorLeft->GetSelectedSensorPosition(0) - (iStartPos + iFloorToSwitch));
	if ((fCurPos) > iFloorToSwitch)
	{
		// Divides the current elevator position by distance between switch and scale iff elevator is above switch height
		fPercent = fLimitLow / (float)(iSwitchToScale);
	}
	if (fPercent > 1.0)
	{
		fPercent = 1.0;
	}
	return fPercent;
}
