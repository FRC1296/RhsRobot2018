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

	pLeftMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
	pRightMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
	pLeftMotor->Set(ControlMode::PercentOutput, 0);
	pRightMotor->Set(ControlMode::PercentOutput, 0);
	pLeftMotor->ConfigOpenloopRamp(.1,0);
	pRightMotor->ConfigOpenloopRamp(.1,0);
	pRightMotor->SetInverted(true);

	pLeftMotor->SetNeutralMode(NeutralMode::Brake);
	pRightMotor->SetNeutralMode(NeutralMode::Brake);
	pIdgey = new PigeonIMU(CAN_PIGEON);
	pIdgey->SetFusedHeading(0.0,10);

	pLeftSlave1->Follow(*pLeftMotor);
	pLeftSlave2->Follow(*pLeftMotor);
	pRightSlave1->Follow(*pRightMotor);
	pRightSlave2->Follow(*pRightMotor);
	pRightSlave1->SetInverted(true);
	pRightSlave2->SetInverted(true);

	fInitRotation = 0;
	fPrevP = 0;
	fSpeed = 0;
	fCurrentPos = 0;
	fTarget = 0;
	fP = 0;
	fD = 0;
	fI = 0;
	fMaxTurnSpeed = 0;
	fMaxStraightSpeed = 0;
	fTurnTTM = 0;
	fStraightTTM = 0;

	iTurnState = -1;
	iTicks = 0;
	iFinalPosLeft = 0;
	iFinalPosRight = 0;

	pPIDTimer = new Timer();
	pSpeedTimer = new Timer();

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
	double dps[3];

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

	SmartDashboard::PutString("Modes","Running");

	SmartDashboard::PutNumber("Left Encoder",pLeftMotor->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("Right Encoder",pRightMotor->GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber("Max Turn Speed",fMaxTurnSpeed);
	SmartDashboard::PutNumber("Max Straight Speed",fMaxStraightSpeed);
	SmartDashboard::PutNumber("Turn TTM",fTurnTTM);
	SmartDashboard::PutNumber("Straight TTM",fStraightTTM);

	if (std::abs(pLeftMotor->GetSelectedSensorVelocity(0)) > fMaxStraightSpeed)
		fMaxStraightSpeed = std::abs(pLeftMotor->GetSelectedSensorVelocity(0));

	pIdgey->GetRawGyro(dps);
	if (std::abs(dps[2]) > fMaxTurnSpeed)
		fMaxTurnSpeed = std::abs(deg[2]);

	/*if (iTurnState == -1){
	fInitRotation = 0;
	}*/

	SmartDashboard::PutNumber("Target Angle",fInitRotation + fTarget);

	if (iTurnState == 7)
	{
		if ((deg[2] <= (fInitRotation + fTarget + 1)) && (deg[2] >= (fInitRotation + fTarget - 1)))
		{
			pPIDTimer->Start();
			if (pPIDTimer->Get() >= .25) {
				SmartDashboard::PutString("Modes","PID Done");
				//SmartDashboard::PutString("Completed","PID Completed");
				pLeftMotor->Set(ControlMode::PercentOutput,0);
				pRightMotor->Set(ControlMode::PercentOutput,0);
				iTurnState = -1;
				fTarget = 0;
				fInitRotation = 0;
				return;
			}
		}
		else
		{
			pPIDTimer->Stop();
			pPIDTimer->Reset();
		}
		SmartDashboard::PutString("Modes","PID Running");
		fP = (fTarget + fInitRotation) - deg[2];
		SmartDashboard::PutNumber("P Value",fP);
		if  (fPrevP == 0)
			fD = 0;
		else
			fD = fPrevP - fP;
		SmartDashboard::PutNumber("D Value",fD);
		//	fI += (DRIVETRAIN_CONST_KP*fP);
		//	SmartDashboard::PutNumber("I Value",fI);
		fPrevP = fP;
		SmartDashboard::PutNumber("Previous P Value",fPrevP);
		fSpeed = (DRIVETRAIN_CONST_KP*fP) /*+ (DRIVETRAIN_CONST_KI*fI)*/ - (DRIVETRAIN_CONST_KD*fD);
		if (fSpeed < -1)
			fSpeed = -1;
		if (fSpeed > 1)
			fSpeed = 1;
		SmartDashboard::PutNumber("Speed",fSpeed);
		pLeftMotor->Set(ControlMode::PercentOutput,fSpeed);
		pRightMotor->Set(ControlMode::PercentOutput,-1*fSpeed);
		SmartDashboard::PutString("Completed","PID Not Completed");
	}

	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_DRIVETRAIN_RUN_ARCADE:
		if (iTurnState == -1) {
			pLeftMotor->Set(ControlMode::PercentOutput,localMessage.params.adrive.left);
			pRightMotor->Set(ControlMode::PercentOutput,localMessage.params.adrive.right);
		}
		break;

	case COMMAND_DRIVETRAIN_WAVE:
		//y=10sin((3.14/50)+C_2)+50
		y = (25*sin(PI/50 * x) + 75)/100.0;
		z = (25*sin((PI/50 * x)-PI) + 75)/100.0;
		pLeftMotor->Set(ControlMode::PercentOutput,-1*y);
		pRightMotor->Set(ControlMode::PercentOutput,z);
		x++;
		break;

	case COMMAND_DRIVETRAIN_GPTURN:
		if (iTurnState == -1)
		{
			fInitRotation = deg[2];
			iTurnState = 7;
			fTarget = (localMessage.params.turn.fAngle);
			SmartDashboard::PutString("Modes","PID Turn Initiated");
		}
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
