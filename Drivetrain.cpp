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
#include "RobotMessage.h"
#include "RhsRobotBase.h"

#define DISTANCE 5 //robotMessage.params.mmove.fDistance
#define WIDTH 26
#define DEGREES 90 //robotMessage.params.turn.fAngle

//Robot

Drivetrain::Drivetrain()
: ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE, DRIVETRAIN_PRIORITY)
{
	//TODO: add member objects
	pLeftMotor = new TalonSRX(CAN_DRIVETRAIN_TALON_LEFT);
	pRightMotor = new TalonSRX(CAN_DRIVETRAIN_TALON_RIGHT);

	pLeftSlave1 = new VictorSPX(CAN_DRIVETRAIN_VICTOR_LEFT1);
	pLeftSlave2 = new VictorSPX(CAN_DRIVETRAIN_VICTOR_LEFT2);
	pRightSlave1 = new VictorSPX(CAN_DRIVETRAIN_VICTOR_RIGHT1);
	pRightSlave2 = new VictorSPX(CAN_DRIVETRAIN_VICTOR_RIGHT2);

	/*	pLeftMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
	pRightMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0); */
	pLeftMotor->Set(ControlMode::PercentOutput, 0);
	pRightMotor->Set(ControlMode::PercentOutput, 0);
	pLeftMotor->ConfigOpenloopRamp(.1,0);
	pRightMotor->ConfigOpenloopRamp(.1,0);
	pRightMotor->SetInverted(true);

	pLeftMotor->SetNeutralMode(NeutralMode::Brake);
	pRightMotor->SetNeutralMode(NeutralMode::Brake);
	pIdgey = new PigeonIMU(pRightMotor);
	pIdgey->SetFusedHeading(0.0,10);

	pLeftSlave1->Follow(*pLeftMotor);
	pLeftSlave2->Follow(*pLeftMotor);
	pRightSlave1->Follow(*pRightMotor);
	pRightSlave2->Follow(*pRightMotor);
	pRightSlave1->SetInverted(true);
	pRightSlave2->SetInverted(true);

	fInitRotation = 0;
	iTurnState = -1;
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
	static int x = 0;
	double y;
	double z;
	double deg[3];

	PigeonIMU::GeneralStatus genStatus;

	pIdgey->GetGeneralStatus(genStatus);
	pIdgey->GetAccumGyro(deg);

	/*PigeonIMU::FusionStatus *stat = new PigeonIMU::FusionStatus();
	pIdgey->GetFusedHeading(*stat);

	SmartDashboard::PutString("Pigeon",stat->description);*/
	SmartDashboard::PutNumber("X Rotation",deg[0]);
	SmartDashboard::PutNumber("Y Rotation",deg[1]);
	SmartDashboard::PutNumber("Z Rotation",deg[2]);
	SmartDashboard::PutNumber("fInite",fInitRotation);

	if (iTurnState == 2){
		if (deg[2] < fInitRotation + 90.0) {
			//pIdgey->GetAccumGyro(deg);
			pLeftMotor->Set(ControlMode::PercentOutput,.2);
			pRightMotor->Set(ControlMode::PercentOutput,-.2);
			return;
		}

		else {
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = -1;
		}
	}
	if (iTurnState == -2){
		if (deg[2] > fInitRotation - 90.0) {
			pLeftMotor->Set(ControlMode::PercentOutput,-.2);
			pRightMotor->Set(ControlMode::PercentOutput,.2);
			return;
		}

		else {
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = -1;
		}
	}
	if (iTurnState == 5){
		if (deg[2] < fInitRotation + 180.0) {
			pLeftMotor->Set(ControlMode::PercentOutput,.2);
			pRightMotor->Set(ControlMode::PercentOutput,-.2);
			return;
		}

		else {
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = -1;
		}
	}
	/*if (iTurnState == -1){
	fInitRotation = 0;
	}*/
	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_DRIVETRAIN_RUN_ARCADE:
		pLeftMotor->Set(ControlMode::PercentOutput,localMessage.params.adrive.left);
		pRightMotor->Set(ControlMode::PercentOutput,localMessage.params.adrive.right);
		break;

	case COMMAND_DRIVETRAIN_WAVE:
		//y=10sin((3.14/50)+C_2)+50
		y = (50*sin(PI/50 * x))/100.0;
		z = (50*sin((PI/50 * x)-PI))/100.0;
		pLeftMotor->Set(ControlMode::PercentOutput,y);
		pRightMotor->Set(ControlMode::PercentOutput,z);
		x++;
		break;

	case COMMAND_DRIVETRAIN_LEFT90:

		//if (fInitRotation == 0) {
		fInitRotation = deg[2];
		iTurnState = 2;
		//pIdgey->GetAccumGyro(deg);
		//}

		break;

	case COMMAND_DRIVETRAIN_RIGHT90:
		fInitRotation = deg[2];
		iTurnState = -2;
		break;

	case COMMAND_DRIVETRAIN_180:
		fInitRotation = deg[2];
		iTurnState = 5;
		break;

		/*		case COMMAND_DRIVETRAIN_MMOVE:
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
			break; */

	default:
		break;
	}
};
