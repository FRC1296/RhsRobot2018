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
#include "Arm.h"
#include "RobotMessage.h"

//Robot

Claw::Claw()
: ComponentBase(CLAW_TASKNAME, CLAW_QUEUE, CLAW_PRIORITY)
{
	pClawVictorLeft = new VictorSPX(CAN_CLAW_VICTOR_LEFT);
	pClawVictorRight = new VictorSPX(CAN_CLAW_VICTOR_RIGHT);
	pClawVictorLeft->SetNeutralMode(NeutralMode::Brake);
	pClawVictorRight->SetNeutralMode(NeutralMode::Brake);

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
	switch(localMessage.command)			//Reads the message command
	{
		case COMMAND_COMPONENT_TEST:
			break;

		case COMMAND_SYSTEM_PDBDATA:
			if ((localMessage.params.pdb.lclaw > CLAW_LIMIT) ||
					(localMessage.params.pdb.rclaw > CLAW_LIMIT))
			{
				pClawVictorLeft->Set(ControlMode::PercentOutput, 0.0);
				pClawVictorRight->Set(ControlMode::PercentOutput, 0.0);
				motorsStopped = true;
			}
			else
			{
				motorsStopped = false;
			}
			break;

		case COMMAND_CLAW_INHALE:
			if(motorsStopped == false)
			{
				pClawVictorLeft->Set(ControlMode::PercentOutput,localMessage.params.claw.fClawSpeed);
				pClawVictorRight->Set(ControlMode::PercentOutput,(localMessage.params.claw.fClawSpeed)*-1);
			}
			break;

		case COMMAND_CLAW_EXHALE:
			if(motorsStopped == false)
			{
				pClawVictorLeft->Set(ControlMode::PercentOutput,(localMessage.params.claw.fClawSpeed)*-1);
				pClawVictorRight->Set(ControlMode::PercentOutput,localMessage.params.claw.fClawSpeed);
			}
			break;

		case COMMAND_CLAW_STOP:
			pClawVictorLeft->Set(ControlMode::PercentOutput, 0.0);
			pClawVictorRight->Set(ControlMode::PercentOutput, 0.0);
			motorsStopped = false;
			break;

		default:
			break;
		}
};
