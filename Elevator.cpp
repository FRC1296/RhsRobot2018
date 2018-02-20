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

	pElevatorMotorLeft->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative,0,0);
	pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
	pElevatorMotorRight->Set(ControlMode::PercentOutput,0);
	pElevatorMotorLeft->EnableCurrentLimit(true);
	pElevatorMotorRight->EnableCurrentLimit(true);
	pElevatorMotorLeft->ConfigContinuousCurrentLimit(5,0);
	pElevatorMotorRight->ConfigContinuousCurrentLimit(5,0);

	pElevatorMotorRight->Set(ControlMode::Follower,CAN_ELEVATOR_TALON_LEFT);

	iLeftInit = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	/*pElevatorMotorLeft->SetSelectedSensorPosition(iLeftInit,0,0);*/

	pElevatorMotorRight->SetInverted(true);

	fCurVoltage = 0;
	fMotorSpeed = 0;

	iTicks = pElevatorMotorLeft->GetSelectedSensorPosition(0);

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

	SmartDashboard::PutNumber("Elevator Speed",fMotorSpeed);
	iTicks = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	SmartDashboard::PutNumber("Elevator Position",iTicks);

	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		case COMMAND_ELEVATOR_MOVE: //{
			fMotorSpeed = .65*(localMessage.params.elevator.fSpeed);
			if (fMotorSpeed > .65)
				fMotorSpeed = .65;
			if (fMotorSpeed < -.65)
				fMotorSpeed = -.65;
			pElevatorMotorLeft->Set(ControlMode::PercentOutput,fMotorSpeed);
/*			int iElevatorDiff = (iFloorToMax / 5) * fCurVoltage;
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iElevatorDiff);
			pElevatorMotorRight->Set(ControlMode::Position,iRightInit - iElevatorDiff); */
			break;
//		}

		case COMMAND_ELEVATOR_FLOOR:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit);
			break;

		case COMMAND_ELEVATOR_SWITCH:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToSwitch);
			break;

		case COMMAND_ELEVATOR_SCALE_LOW:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToLowScale);
			break;

		case COMMAND_ELEVATOR_SCALE_MID:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToMidScale);
			break;

		case COMMAND_ELEVATOR_SCALE_HIGH:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToHighScale);
			break;

		case COMMAND_ELEVATOR_CLIMB:
			pElevatorMotorLeft->Set(ControlMode::Position,iLeftInit + iFloorToClimb);
			break;

		default:
			break;
		}
};
