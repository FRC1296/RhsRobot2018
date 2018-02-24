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
#include "RobotMessage.h"

//Robot

Claw::Claw()
: ComponentBase(CLAW_TASKNAME, CLAW_QUEUE, CLAW_PRIORITY)
{
	//TODO: add member objects
	pClawVictorLeft = new VictorSPX(CAN_CLAW_VICTOR_LEFT);
	pClawVictorRight = new VictorSPX(CAN_CLAW_VICTOR_RIGHT);
	pClawSolenoidLeft = new Solenoid(CAN_PCM, 0);
	pClawSolenoidRight = new Solenoid(CAN_PCM, 1);
	pClawSolenoidLeft->Set(true);
	pClawSolenoidRight->Set(true);

	pPDP = new PowerDistributionPanel(CAN_PDB);

	motorsStopped = false;

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
	if (motorsStopped && !(pPDP->GetCurrent(CLAW_CHANNEL_ONE) > CLAW_LIMIT || pPDP->GetCurrent(CLAW_CHANNEL_TWO) > CLAW_LIMIT))
		motorsStopped = false;

	SmartDashboard::PutNumber("Claw Current",pPDP->GetCurrent(CLAW_CHANNEL_ONE));

	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		case COMMAND_CLAW_INHALE:
			pClawVictorLeft->Set(ControlMode::PercentOutput,localMessage.params.claw.fClawSpeed);
			pClawVictorRight->Set(ControlMode::PercentOutput,(localMessage.params.claw.fClawSpeed)*-1);
			SmartDashboard::PutNumber("LeftTrigger",L310_TRIGGER_LEFT);
			SmartDashboard::PutNumber("RightTrigger",L310_TRIGGER_RIGHT);
			if (!(motorsStopped) && (pPDP->GetCurrent(CLAW_CHANNEL_ONE) > CLAW_LIMIT || pPDP->GetCurrent(CLAW_CHANNEL_TWO) > CLAW_LIMIT))
			{
				pClawVictorLeft->Set(ControlMode::PercentOutput,0);
				pClawVictorRight->Set(ControlMode::PercentOutput,0);
				motorsStopped = true;
			}
			break;

		case COMMAND_CLAW_EXHALE:
			pClawVictorLeft->Set(ControlMode::PercentOutput,(localMessage.params.claw.fClawSpeed)*-1);
			pClawVictorRight->Set(ControlMode::PercentOutput,localMessage.params.claw.fClawSpeed);
			break;

		case COMMAND_CLAW_PINCH:
			pClawSolenoidLeft->Set(true);
			pClawSolenoidRight->Set(false);
			break;

		case COMMAND_CLAW_RELEASE:
			pClawSolenoidLeft->Set(false);
			pClawSolenoidRight->Set(true);
			break;

		case COMMAND_CLAW_STOP:
			pClawVictorLeft->Set(ControlMode::PercentOutput,0);
			pClawVictorRight->Set(ControlMode::PercentOutput,0);
			break;

		default:
			break;
		}
};
