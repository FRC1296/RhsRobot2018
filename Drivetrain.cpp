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
#include "Drivetrain.h"

#define DISTANCE 5 //robotMessage.params.mmove.fDistance
#define WIDTH 26
#define DEGREES 90 //robotMessage.params.turn.fAngle

//Robot

Drivetrain::Drivetrain()
: ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE, DRIVETRAIN_PRIORITY)
{
	//TODO: add member objects
	pLeftMotor = new TalonSRX(CAN_DRIVETRAIN_LEFT);
	pRightMotor = new TalonSRX(CAN_DRIVETRAIN_RIGHT);

	pLeftMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
	pRightMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);

	iTicks = 0;
	iFinalPosLeft = 0;
	iFinalPosRight = 0;



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

		case COMMAND_DRIVETRAIN_MMOVE:
			iTicks = (DISTANCE*4096)/(PI*DIAMETER);
			iFinalPosLeft = iTicks + pLeftMotor->GetSelectedSensorPosition(0);
			iFinalPosRight = iTicks + pRightMotor->GetSelectedSensorPosition(0);
			pLeftMotor->Set(ControlMode::Position,iFinalPosLeft);
			pRightMotor->Set(ControlMode::Position,iFinalPosRight);
			break;

		case COMMAND_DRIVETRAIN_MTURN:
			iTicks = (512*WIDTH*DEGREES)/(45*DIAMETER);
			if (DEGREES<0) {
				iFinalPosLeft = pLeftMotor->GetSelectedSensorPosition(0) - iTicks;
				iFinalPosRight = pRightMotor->GetSelectedSensorPosition(0) + iTicks;
			}
			else {
				iFinalPosLeft = pLeftMotor->GetSelectedSensorPosition(0) + iTicks;
				iFinalPosRight = pRightMotor->GetSelectedSensorPosition(0) - iTicks;
			}
			pLeftMotor->Set(ControlMode::Position,iFinalPosLeft);
			pRightMotor->Set(ControlMode::Position,iFinalPosRight);
			break;

		default:
			break;
	}
};
