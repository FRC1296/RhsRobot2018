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
	pElevatorMotor = new TalonSRX(CAN_ELEVATOR_TALON);

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

		default:
			break;
		}
};
