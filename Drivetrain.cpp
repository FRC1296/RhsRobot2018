/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */

#include "WPILib.h"

#include "Component.h"
#include "ComponentBase.h"
#include "RobotParams.h"
#include "Drivetrain.h"

//Robot

Drivetrain::Drivetrain()
: ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE, DRIVETRAIN_PRIORITY)
{
	//TODO: add member objects
	pLeftMotor = new WPI_TalonSRX(CAN_DRIVETRAIN_LEFT);
	pRightMotor = new WPI_TalonSRX(CAN_DRIVETRAIN_RIGHT);
	pTask = new std::thread(&Drivetrain::StartTask, this, DRIVETRAIN_TASKNAME, DRIVETRAIN_PRIORITY);
	wpi_assert(pTask);
};

Drivetrain::~Drivetrain()
{
	//TODO delete member objects
	delete(pTask);
};

void Drivetrain::OnStateChange()
{
};

void Drivetrain::Run()
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
