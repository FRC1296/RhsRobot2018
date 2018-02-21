/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */

#include "Component.h"
#include "ComponentBase.h"
#include "RobotParams.h"
#include "WPILib.h"
#include "Elevator.h"

//Robot

Elevator::Elevator()
: ComponentBase(ELEVATOR_TASKNAME, ELEVATOR_QUEUE, ELEVATOR_PRIORITY)
{
	//TODO: add member objects
	pElevatorMotorLeft = new TalonSRX(CAN_ELEVATOR_TALON_LEFT);
	pElevatorMotorRight = new TalonSRX(CAN_ELEVATOR_TALON_RIGHT);

	pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
	pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);

	pElevatorMotorLeft->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
	pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
	pElevatorMotorRight->Set(ControlMode::PercentOutput,0);
	pElevatorMotorLeft->EnableCurrentLimit(true);
	pElevatorMotorRight->EnableCurrentLimit(true);
	pElevatorMotorLeft->ConfigContinuousCurrentLimit(30,0);
	pElevatorMotorRight->ConfigContinuousCurrentLimit(30,0);
	pElevatorMotorLeft->Config_kP(0,0.05,5000);
	pElevatorMotorLeft->Config_kI(0,0,10);
	pElevatorMotorLeft->Config_kD(0,0.3,5000);

	pElevatorMotorLeft->SelectProfileSlot(0,0);

	pElevatorMotorLeft->ConfigPeakOutputForward(.5,0);
	pElevatorMotorLeft->ConfigPeakOutputReverse(-.5,0);

	pElevatorMotorRight->Set(ControlMode::Follower,CAN_ELEVATOR_TALON_LEFT);

	iLeftInit = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	/*pElevatorMotorLeft->SetSelectedSensorPosition(iLeftInit,0,0);*/

	pElevatorMotorRight->SetInverted(true);
	//pElevatorMotorLeft->SetInverted(true);

	fCurVoltage = 0;
	fMotorSpeed = 0;
	fMaxSpeed = 0;

	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	iStartPos = iCurrPos;
	iStopPos = (iStartPos + LIFT_STOP_POS);
	iEleState = EleState_Init;

	//pElevatorMotorLeft->SetSensorPhase(true);

	pEleTimer = new Timer();
	pEleTimeout = new Timer();

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

	SmartDashboard::PutNumber("Elevator Speed RPM",((fMaxSpeed*600)/4096.0));
	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);

	static double CurrMotorLeft = 0;// = pElevatorMotorLeft->GetOutputCurrent();
	SmartDashboard::PutNumber("Elevator Current",CurrMotorLeft);

	SmartDashboard::PutNumber("Elevator Init Position",iStartPos);
	SmartDashboard::PutNumber("Elevator Switch Position",iStartPos + iFloorToSwitch);
	SmartDashboard::PutNumber("Elevator Scale Position",iStartPos + iFloorToScale);
	SmartDashboard::PutNumber("Difference",(iStartPos + iFloorToScale) - iCurrPos);

	/*if(CurrMotorLeft<pElevatorMotorLeft->GetOutputCurrent())
	{
		CurrMotorLeft = pElevatorMotorLeft->GetOutputCurrent();
	}

	if (fMaxSpeed > pElevatorMotorLeft->GetSelectedSensorVelocity(0))
	{
		fMaxSpeed = pElevatorMotorLeft->GetSelectedSensorVelocity(0);
	}*/
	SmartDashboard::PutNumber("Elevator Speed",fMaxSpeed);

	switch(iEleState)
	{
	case EleState_Init:
		SmartDashboard::PutString("Elevator State","Elevator Init");
		break;
	case EleState_Scale:
		SmartDashboard::PutString("Elevator State","Scale Mode");
		Scale(iCurrPos);
		break;
	case EleState_Switch:
		SmartDashboard::PutString("Elevator State","Switch Mode");
		Switch(iCurrPos);
		break;
	case EleState_Floor:
		SmartDashboard::PutString("Elevator State","Floor Mode");
		Floor(iCurrPos);
		break;
	}

	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_ELEVATOR_MOVE: //{
		if (iEleState == EleState_Init)
		{
			fMotorSpeed = 1*(localMessage.params.elevator.fSpeed);
			if (fMotorSpeed > 1)
			{
				fMotorSpeed = 1;
			}
			if (fMotorSpeed < -1)
			{
				fMotorSpeed = -1;
			}
	/*		if (iCurrPos <= iStopPos && fMotorSpeed < 0){
				fMotorSpeed = 0;
			}
			if (iCurrPos >= iStartPos && fMotorSpeed > 0){
				fMotorSpeed = 0;
			} */
		//	pElevatorMotorLeft->Set(ControlMode::PercentOutput,fMotorSpeed);
			/*			int iElevatorDiff = (iFloorToMax / 5) * fCurVoltage;
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iElevatorDiff);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iElevatorDiff); */
		}
		break;

	case COMMAND_ELEVATOR_FLOOR:
		if (iEleState == EleState_Init)
		{
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos);
			iEleState = EleState_Floor;
			SmartDashboard::PutString("Button", "A Button Pushed");
			pEleTimeout->Reset();
			pEleTimeout->Start();
			pEleTimer->Reset();
		}
		break;

	case COMMAND_ELEVATOR_SWITCH:
		if (iEleState == EleState_Init)
		{
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToSwitch);
			iEleState = EleState_Switch;
			SmartDashboard::PutString("Button", "B Button Pushed");
			pEleTimeout->Reset();
			pEleTimeout->Start();
			pEleTimer->Reset();
		}
		break;

	case COMMAND_ELEVATOR_SCALE:
		if (iEleState == EleState_Init)
		{
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToScale);
			iEleState = EleState_Scale;
			SmartDashboard::PutString("Button", "Y Button Pushed");
			pEleTimeout->Reset();
			pEleTimeout->Start();
			pEleTimer->Reset();
		}
		break;

	case COMMAND_ELEVATOR_CLIMB:
	//	pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToClimb);
		break;

	default:
		if(iEleState == EleState_Init)
		{
			pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		}
		break;
	}
};

void Elevator::Floor(int iCurrPos) {
	if (pEleTimeout->Get() > 3)
	{
		pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		iEleState = EleState_Init;
		pEleTimeout->Stop();
		//pEleTimeout->Reset();
		return;
	}
	if ((iCurrPos <= iStartPos + ACCEPT_RANGE_ELE) && (iCurrPos >= iStartPos - ACCEPT_RANGE_ELE))
	{
		pEleTimer->Start();
		if (pEleTimer->Get() > .25)
		{
			iEleState = EleState_Init;
			pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		}
	}
	else
	{
		pEleTimer->Stop();
		pEleTimer->Reset();
	}
}

void Elevator::Switch(int iCurrPos) {
	if (pEleTimeout->Get() > 3)
	{
		pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		iEleState = EleState_Init;
		pEleTimeout->Stop();
		//pEleTimeout->Reset();
		return;
	}
	if ((iCurrPos <= ((iStartPos + iFloorToSwitch) + ACCEPT_RANGE_ELE)) && (iCurrPos >= ((iStartPos + iFloorToSwitch) - ACCEPT_RANGE_ELE)))
	{
		pEleTimer->Start();
		if (pEleTimer->Get() > .25)
		{
			iEleState = EleState_Init;
			pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		}
	}
	else
	{
		pEleTimer->Stop();
		pEleTimer->Reset();
	}
}

void Elevator::Scale(int iCurrPos) {
	if (pEleTimeout->Get() > 3)
	{
		pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		iEleState = EleState_Init;
		pEleTimeout->Stop();
		//pEleTimeout->Reset();
		return;
	}
	if ((iCurrPos <= ((iStartPos + iFloorToScale) + ACCEPT_RANGE_ELE)) && (iCurrPos >= ((iStartPos + iFloorToScale) - ACCEPT_RANGE_ELE)))
	{
		pEleTimer->Start();
		if (pEleTimer->Get() > .25)
		{
			iEleState = EleState_Init;
			pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
		}
	}
	else
	{
		pEleTimer->Stop();
		pEleTimer->Reset();
	}
}
