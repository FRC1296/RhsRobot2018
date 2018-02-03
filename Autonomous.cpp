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
#include "Autonomous.h"

//Robot

Autonomous::Autonomous()
: ComponentBase(AUTONOMOUS_TASKNAME, AUTONOMOUS_QUEUE, AUTONOMOUS_PRIORITY)
{
	//TODO: add member objects
	pTask = new std::thread(&Component::StartTask, this, AUTONOMOUS_TASKNAME, AUTONOMOUS_PRIORITY);
	wpi_assert(pTask);
};

Autonomous::~Autonomous()
{
	//TODO delete member objects

	delete(pTask);
};

void Autonomous::OnStateChange()
{
};

void Autonomous::Run()
{
	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Autonomous
	/*	case COMMAND_AUTONOMOUS_TEST:
			break;
*/
		default:
			break;
		}
};
