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
	pElevatorMotorRight->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
	pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
	pElevatorMotorRight->Set(ControlMode::PercentOutput,0);
	pElevatorMotorLeft->EnableCurrentLimit(true);
	pElevatorMotorRight->EnableCurrentLimit(true);
	pElevatorMotorLeft->ConfigContinuousCurrentLimit(20,0);
	pElevatorMotorRight->ConfigContinuousCurrentLimit(20,0);

	iLeftInit = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	iRightInit = pElevatorMotorRight->GetSelectedSensorPosition(0);

	fCurVoltage = 0;

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
	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		case COMMAND_ELEVATOR_MOVE: {
			int iElevatorDiff = (iFloorToMax / 5) * fCurVoltage;
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iElevatorDiff);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iElevatorDiff);
			break;
		}

		case COMMAND_ELEVATOR_FLOOR:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit);
			break;

		case COMMAND_ELEVATOR_SWITCH:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToSwitch);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iFloorToSwitch);
			break;

		case COMMAND_ELEVATOR_SCALE_LOW:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToLowScale);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iFloorToLowScale);
			break;

		case COMMAND_ELEVATOR_SCALE_MID:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToMidScale);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iFloorToMidScale);
			break;

		case COMMAND_ELEVATOR_SCALE_HIGH:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToHighScale);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iFloorToHighScale);
			break;

		case COMMAND_ELEVATOR_CLIMB:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToClimb);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iFloorToClimb);
			break;

		default:
			break;
		}
};
