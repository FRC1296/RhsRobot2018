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
#include "Claw.h"

//Robot

Claw::Claw()
: ComponentBase(CLAW_TASKNAME, CLAW_QUEUE, CLAW_PRIORITY)
{
	//TODO: add member objects
	pClawVictorLeft = new VictorSPX(CAN_CLAW_VICTOR_LEFT);
	pClawVictorRight = new VictorSPX(CAN_CLAW_VICTOR_RIGHT);

	pTask = new std::thread(&Component::StartTask, this, CLAW_TASKNAME, CLAW_PRIORITY);
	wpi_assert(pTask);
};

Claw::~Claw()
{
	//TODO delete member objects
	delete(pTask);
};

void Claw::OnStateChange()
{
};

void Claw::Run()
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
