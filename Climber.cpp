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
#include "Climber.h"

//Robot

Climber::Climber()
: ComponentBase(CLIMBER_TASKNAME, CLIMBER_QUEUE, CLIMBER_PRIORITY)
{
	pUpMotor = new TalonSRX(CAN_CLIMBER_TALON);
	pUpSlave = new VictorSPX(CAN_CLIMBER_VICTOR);
	//TODO: add member objects
	pTask = new std::thread(&Component::StartTask, this, COMPONENT_TASKNAME, COMPONENT_PRIORITY);
	wpi_assert(pTask);

	pUpMotor->Set(ControlMode::PercentOutput,0);
	pUpSlave->Follow(*pUpMotor);
	pUpMotor->EnableCurrentLimit(true);
	pUpMotor->ConfigContinuousCurrentLimit(30,0);
};

Climber::~Climber()
{
	//TODO delete member objects
	delete(pTask);
};

void Climber::OnStateChange()
{
};

void Component::Run()
{
	switch(localMessage.command)			//Reads the message command
	{

	//When push le left bumper, set control mode percent output 50%

	//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		default:
			break;
		}
};
