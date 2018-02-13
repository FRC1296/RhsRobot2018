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
#define ROBOT_WIDTH 26
#define DEGREES 90 //robotMessage.params.turn.fAngle

enum TurnState{ TurnState_Init = -1, TurnState_mTurn = 1, TurnState_gpTurn, TurnState_boxTurn};
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
	fMaxTurnX = 0;
	fMaxTurnY = 0;
	fMaxTurnZ = 0;
	fTimeToDest = 0;

	dAvgArray1 = 0;
	dAvgArray2 = 0;

	iTurnState = TurnState_Init;
	iTicks = 0;
	iFinalPosLeft = 0;
	iFinalPosRight = 0;
	memset(iTurnArray,0,sizeof(int)*FILTER_ONE_LENGTH);
	memset(dTurnArray2,0,sizeof(float)*FILTER_TWO_LENGTH);

	iNumPoints = 0;
	iCurrNumPoints = 0;

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

	pIdgey->GetRawGyro(dps);
	if (std::abs(dps[2]) > fMaxTurnSpeed)
		fMaxTurnSpeed = std::abs(dps[2]);
	if (std::abs(dps[0]) > fMaxTurnX)
		fMaxTurnX = std::abs(dps[0]);
	if (std::abs(dps[1]) > fMaxTurnY)
		fMaxTurnY = std::abs(dps[1]);
	if (std::abs(dps[2]) > fMaxTurnZ)
		fMaxTurnZ = std::abs(dps[2]);

	/*if (iTurnState == -1){
	fInitRotation = 0;
	}*/

	SmartDashboard::PutNumber("Target Angle",fInitRotation + fTarget);

	switch (iTurnState)
	{
	case TurnState_mTurn:
		MeasuredTurn();
		break;
	case TurnState_gpTurn:
		GyroPIDTurn();
		break;
	case TurnState_boxTurn:
		BoxCarFilter();
		break;
	default:
		break;
	}

	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_DRIVETRAIN_RUN_ARCADE:
		if (iTurnState == TurnState_Init) {
			SmartDashboard::PutString("Mode","ARCADEEEEEE");
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
		SmartDashboard::PutString("Command","Gracious Professionalism Turn");
		if (iTurnState == TurnState_Init)
		{
			fTarget = (localMessage.params.turn.fAngle);
			iTicks = (512*ROBOT_WIDTH*fTarget)/(45*WHEEL_DIA);
			SmartDashboard::PutNumber("ticks",iTicks);
			fTimeToDest = (iTicks*1.0/MAX_TURN_SPEED);
			SmartDashboard::PutNumber("TTD",fTimeToDest);
			iNumPoints = std::abs(fTimeToDest/UPDATE_RATE);
			SmartDashboard::PutNumber("NmPts",iNumPoints);
			iTurnState = TurnState_gpTurn;
			fInitRotation = deg[2];
			iCurrNumPoints = 0;

			SmartDashboard::PutString("Modes","PID Turn Initiated");
		}
		break;
	case COMMAND_DRIVETRAIN_MMOVE:
		if (iTurnState == TurnState_Init)
		{
			iTicks = (DISTANCE*4096)/(PI*WHEEL_DIA);
			iFinalPosLeft = iTicks + pLeftMotor->GetSelectedSensorPosition(0);
			iFinalPosRight = iTicks + pRightMotor->GetSelectedSensorPosition(0);
			pLeftMotor->Set(ControlMode::Position,iFinalPosLeft);
			pRightMotor->Set(ControlMode::Position,iFinalPosRight);
		}
		break;

	case COMMAND_DRIVETRAIN_MTURN:
		if (iTurnState == TurnState_Init)
		{
			fTarget = (localMessage.params.turn.fAngle);
			iTicks = (512*ROBOT_WIDTH*fTarget)/(45*WHEEL_DIA);
			/*fTimeToDest = ((1.0*iTicks)/(MAX_TURN_SPEED));
			iNumPoints = (fTimeToDest/UPDATE_RATE);*/
			iTurnState = TurnState_mTurn;
			if (fTarget<0) {
				iFinalPosLeft = pLeftMotor->GetSelectedSensorPosition(0) - iTicks;
				iFinalPosRight = pRightMotor->GetSelectedSensorPosition(0) + iTicks;
			}
			else {
				iFinalPosLeft = pLeftMotor->GetSelectedSensorPosition(0) + iTicks;
				iFinalPosRight = pRightMotor->GetSelectedSensorPosition(0) - iTicks;
			}
			pLeftMotor->Set(ControlMode::Position,iFinalPosLeft);
			pRightMotor->Set(ControlMode::Position,iFinalPosRight);
		}
		break;

	default:
		break;
	}


};

void Drivetrain::BoxCarFilter()
{
	float fTargetCalc = fInitRotation + fTarget;

	if ((deg[2] <= (fTargetCalc + ACCEPT_RANGE_DEGR)) && (deg[2] >= (fTargetCalc - ACCEPT_RANGE_DEGR)))
	{
		pPIDTimer->Start();
		if (pPIDTimer->Get() >= .25) {
			SmartDashboard::PutString("snowflake","PID Done");
			SmartDashboard::PutString("Completed","PID Completed");
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = TurnState_Init;
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

	SmartDashboard::PutNumber("Current Point",iCurrNumPoints);
	if (iCurrNumPoints <= iNumPoints)
	{
		iCurrNumPoints ++;
		AddToArray(iTurnArray, FILTER_ONE_LENGTH, MAX_TURN_SPEED);

	}
	else
	{
		AddToArray(iTurnArray, FILTER_ONE_LENGTH, 0);
		//iTurnArray + 0;
		iCurrNumPoints += 1;
	}

	dAvgArray1 = AvgArrays(iTurnArray, FILTER_ONE_LENGTH);
	SmartDashboard::PutNumber("Ave1",dAvgArray1);

	dAddToArray(dTurnArray2, FILTER_TWO_LENGTH, dAvgArray1);
	dAvgArray2 = dAvgArrays(dTurnArray2, FILTER_TWO_LENGTH);
	fSpeed = dAvgArray2;

	if ((dAvgArray1 + dAvgArray2) == 0)
	{
		iTurnState = TurnState_Init;
		SmartDashboard::PutString("Completed","Boxcar Terminated");
	}

	SmartDashboard::PutString("Modes","PID Running");
	SmartDashboard::PutNumber("Speed",fSpeed);
	pLeftMotor->Set(ControlMode::Velocity,fSpeed);
	pRightMotor->Set(ControlMode::Velocity,-1*fSpeed);
	SmartDashboard::PutString("Completed","PID Not Completed");
}

void Drivetrain::GyroPIDTurn()
{
	static float fTargetCalc = fInitRotation + fTarget;

	if ((deg[2] <= (fTargetCalc + ACCEPT_RANGE_DEGR)) && (deg[2] >= (fTargetCalc - ACCEPT_RANGE_DEGR)))
	{
		pPIDTimer->Start();
		if (pPIDTimer->Get() >= .25) {
			SmartDashboard::PutString("Completed","PID Completed");
			pLeftMotor->Set(ControlMode::PercentOutput,0);
			pRightMotor->Set(ControlMode::PercentOutput,0);
			iTurnState = TurnState_Init;
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
	if (fSpeed < -1 * MAX_SPEED_PID)
		fSpeed = -1 * MAX_SPEED_PID;
	if (fSpeed > MAX_SPEED_PID)
		fSpeed = MAX_SPEED_PID;
	SmartDashboard::PutNumber("Speed",fSpeed);
	pLeftMotor->Set(ControlMode::Velocity,fSpeed);
	pRightMotor->Set(ControlMode::Velocity,-1*fSpeed);
	SmartDashboard::PutString("Completed","PID Not Completed");
}

void Drivetrain::MeasuredTurn()
{
	if((pLeftMotor->GetSelectedSensorPosition(0) > iFinalPosLeft - ACCEPT_RANGE_TICKS) && (pLeftMotor->GetSelectedSensorPosition(0) < iFinalPosLeft + ACCEPT_RANGE_TICKS))
	{
		martDashboard::PutString("Completed","PID Completed");
		iTurnState = TurnState_Init;
		fTarget = 0;
		fInitRotation = 0;
		return;
	}
}

float AvgArrays(int* Array, int LengthArr)
{
	float sum = 0;
	int iCurrVal;
	for(iCurrVal = 0; iCurrVal < LengthArr; iCurrVal++)
	{
		sum += Array[iCurrVal];
	}
	sum = (sum/LengthArr);
	return sum;
}

void AddToArray(int* Array, int LengthArr, int val)
{
	//	int i = LengthArr;
	for (int i = LengthArr - 1; i > 0; i--)
	{
		Array[i] = Array[i-1];
	}
	Array[0] = val;
}

float dAvgArrays(float* Array, int LengthArr)
{
	float sum = 0;
	int iCurrVal;
	for(iCurrVal = 0; iCurrVal < LengthArr; iCurrVal++)
	{
		sum += Array[iCurrVal];
	}
	sum = (sum/LengthArr);
	SmartDashboard::PutNumber("Totally not sum :P", sum);
	return sum;
}

void dAddToArray(float* Array, int LengthArr, float val)
{
	//	int i = LengthArr;
	for (int i = LengthArr - 1; i > 0; --i)
	{
		Array[i] = Array[i-1];
	}
	Array[0] = val;
}

