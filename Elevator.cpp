
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
	pElevatorMotorLeft->ConfigPeakOutputForward(1.0, 10);
	pElevatorMotorLeft->ConfigPeakOutputReverse(-.6,10);
	pElevatorMotorRight->ConfigPeakOutputForward(1.0, 10);
	pElevatorMotorRight->ConfigPeakOutputReverse(-0.6,10);

	pElevatorMotorLeft->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,10);
	pElevatorMotorLeft->SetSensorPhase(true);

	pElevatorMotorLeft->Config_kF(0,0.0,10);
	pElevatorMotorLeft->Config_kP(0,0.3,10); // .3
	pElevatorMotorLeft->Config_kI(0,0.0,10);
	pElevatorMotorLeft->Config_kD(0,0.01,10); // .01
	pElevatorMotorLeft->ConfigAllowableClosedloopError(0, 200, 10);

	//pElevatorMotorLeft->Config_kF(1,0.0,10);
	//pElevatorMotorLeft->Config_kP(1,0.2,10);
	//pElevatorMotorLeft->Config_kI(1,0.0,10);
	//pElevatorMotorLeft->Config_kD(1,0.0,10);
	//pElevatorMotorLeft->ConfigAllowableClosedloopError(0, 200, 10);
	pElevatorMotorLeft->Config_kF(1,0.0,10);
	pElevatorMotorLeft->Config_kP(1,0.0,10);
	pElevatorMotorLeft->Config_kI(1,0.10,10);
	pElevatorMotorLeft->ConfigMaxIntegralAccumulator(1, 8092, 10);
	pElevatorMotorLeft->Config_kD(1,0.1,10);
	pElevatorMotorLeft->ConfigAllowableClosedloopError(1, 100, 10);

	iCurrentPid = 0;
	pElevatorMotorLeft->SelectProfileSlot(iCurrentPid,0);

	Wait(1.0);
	pElevatorMotorRight->Set(ControlMode::Follower,CAN_ELEVATOR_TALON_LEFT);
	pElevatorMotorLeft->Set(ControlMode::Position, pElevatorMotorLeft->GetSelectedSensorPosition(0));
	//pElevatorMotorLeft->Set(ControlMode::PercentOutput, 0.0);

	pSlowHallEffect = new DigitalInput(SLOW_HALL_EFFECT_SLOT);
	pStopHallEffect = new DigitalInput(STOP_HALL_EFFECT_SLOT);

	bFloorPos = false;

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

	prevPressed = false;

	prevActivated = false;

//	SmartDashboard::PutNumber("Elevator Speed RPM",((fMaxSpeed*600)/4096.0));
//	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
//	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);

//	SmartDashboard::PutNumber("FIRST Elevator Init Position",iStartPos);
//	SmartDashboard::PutNumber("Elevator Switch Position",iStartPos + iFloorToSwitch);
//	SmartDashboard::PutNumber("Elevator Scale Position",iStartPos + iFloorToScale);
//	SmartDashboard::PutString("Button Pressed", "no button pressed");
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
	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);

	//iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(iCurrentPid);
	iCurrTgt = pElevatorMotorLeft->GetClosedLoopTarget(iCurrentPid);

//	SmartDashboard::PutNumber("New Elevator Init Position",iStartPos);

//	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
//	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);
//	SmartDashboard::PutNumber("MoveDelta",iMoveDelta);

//	SmartDashboard::PutBoolean("Slowdown Hall Effect Sensor",!(pSlowHallEffect->Get()));
//	SmartDashboard::PutBoolean("Stop Hall Effect Sensor",!(pStopHallEffect->Get()));

	if (!prevActivated && !pStopHallEffect->Get()) {
		iStartPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);
		prevActivated = true;
	}
	if (prevActivated && pStopHallEffect->Get()) {
		prevActivated = false;
	}

	if((iCurrTgt - iCurrPos) > 100)
	{
		// change the PIS cojnstants for upward motion
//		SmartDashboard::PutNumber("MoveDelta",iMoveDelta);

		if(iCurrentPid == 1)
		{
			//printf("going up\n");
			;
		}

		iCurrentPid = 0;
	}
	else if((iCurrTgt - iCurrPos) < -100)
	{
		// change the PID constants for downward motion

		if(iCurrentPid == 0)
		{
			//printf("going up\n");
			;
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
//		SmartDashboard::PutString("Button Pressed", "fine adjust");

		if (fMotorSpeed > 0.2 && iCurrPos > iStartPos)
		{
			iMoveDelta -= (iMoveDeltaIncrement + 50);
			iCurrentPid = 0;
			/*
			pElevatorMotorLeft->SetNeutralMode(NeutralMode::Coast);
			pElevatorMotorRight->SetNeutralMode(NeutralMode::Coast);
			 */
		}
		else if(fMotorSpeed < -0.2 && iCurrPos < iStartPos + iFloorToMax)
		{
			iMoveDelta += (iMoveDeltaIncrement);
			iCurrentPid = 0;
		}
		break;

	case COMMAND_ELEVATOR_NOBUTTON:
//		SmartDashboard::PutString("Button Pressed", "no button pressed");
		iMoveDelta = 0;
/*		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Coast);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Coast); */
		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorLeft->Set(ControlMode::Position, iStartPos + iMoveDelta);
		pEleTimeout->Reset();
		pEleTimeout->Start();
//		Zero();
		prevPressed = true;
		break;

	case COMMAND_ELEVATOR_FLOOR:
//		SmartDashboard::PutString("Button Pressed", "Floor Pressed");
		if(!prevPressed) iMoveDelta = 0;
		prevPressed = false;

		if(iPrevDelta == iMoveDelta)
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

		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);

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
//		SmartDashboard::PutString("Button Pressed", "Switch Pressed");

		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
//		SmartDashboard::PutNumber("move delta", iMoveDelta);

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
//		SmartDashboard::PutString("Button Pressed", "Scale Pressed");

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

//		SmartDashboard::PutString("Button", "Y Button Pushed");
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_CLIMB:
		pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
		pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToMax - 750);

		prevPressed = false;
		//	pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToClimb);
		break;

	default:
		prevPressed = false;
		break;
	}

	// implement timeout

//	SmartDashboard::PutNumber("Closed Loop Error", pElevatorMotorLeft->GetClosedLoopError(0));

	if((pElevatorMotorLeft->GetClosedLoopError(0) < 200) && (pElevatorMotorLeft->GetClosedLoopError(0) > -200))
	{
		// the servo has closed
		pEleTimeout->Reset();
		pEleTimeout->Stop();
	}
	else
	{
		if(pEleTimeout->Get() > 2.75)
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

bool Elevator::AtFloorPos()
{
	return !(pStopHallEffect->Get());
}

int Elevator::Zero()
{
	pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iMoveDelta);
	if (!prevPressed) {
		pEleTimeout->Reset();
		pEleTimeout->Start();
	}
	if (pElevatorMotorLeft->GetSelectedSensorPosition(0) > iStartPos + 1296) {
		return iStartPos;
	}
   if (pEleTimeout->Get() < 1.0 && pStopHallEffect->Get()) {
		pElevatorMotorLeft->Set(ControlMode::PercentOutput,-0.1);
		return iStartPos;
	}
	else {
		pElevatorMotorLeft->Set(ControlMode::Position,pElevatorMotorLeft->GetSelectedSensorPosition(0));
		pEleTimeout->Stop();
		pEleTimeout->Reset();
		return pElevatorMotorLeft->GetSelectedSensorPosition(0);
	}
}
