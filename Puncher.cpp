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
#include "Puncher.h"
#include "RobotMessage.h"

//Robot

Puncher::Puncher()
: ComponentBase(PUNCHER_TASKNAME, PUNCHER_QUEUE, PUNCHER_PRIORITY)
{
	//TODO: add member objects
	pTask = new std::thread(&Component::StartTask, this, PUNCHER_TASKNAME, PUNCHER_PRIORITY);
	wpi_assert(pTask);
};

Puncher::~Puncher()
{
	//TODO delete member objects

	delete(pTask);
};

void Puncher::OnStateChange()
{
};

void Puncher::Run()
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
