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

<<<<<<< HEAD
	pClawSolenoidLeft = new Solenoid(CAN_PCM, 0 );
	pClawSolenoidRight = new Solenoid(CAN_PCM, 1);
	pClawSolenoidLeft->Set(true);
	pClawSolenoidRight->Set(true);
=======
	pPDP = new PowerDistributionPanel(CAN_PDB);

	motorsStopped = false;

	/* *************
	 * All the pArmMotor stuff is only in here because of the way the Limit Switch
	 * is currently wired for pneumatics. This may change, but we aren't in charge
	 * of how electrical wires things so RIP.
	 */
>>>>>>> aa710d53e701c5c22cad4869abea0d89b224560f

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

		case COMMAND_CLAW_INHALE:
			pClawVictorLeft->Set(ControlMode::PercentOutput,localMessage.params.claw.fClawSpeed);
			pClawVictorRight->Set(ControlMode::PercentOutput,(localMessage.params.claw.fClawSpeed)*-1);
			break;

		case COMMAND_CLAW_EXHALE:
			pClawVictorLeft->Set(ControlMode::PercentOutput,(localMessage.params.claw.fClawSpeed)*-1);
			pClawVictorRight->Set(ControlMode::PercentOutput,localMessage.params.claw.fClawSpeed);
			break;

		case COMMAND_CLAW_STOP:
			pClawVictorLeft->Set(ControlMode::PercentOutput, 0.0);
			pClawVictorRight->Set(ControlMode::PercentOutput, 0.0);
			break;

		default:
			break;
		}

};
